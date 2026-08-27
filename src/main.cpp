/*
 * ═══════════════════════════════════════════════════════════════
 *  Zb-Rf Board V1.0 — RF Characterization Firmware
 *  CMT2310A  865 MHz  FSK  ESP32-C3
 *  by lohith D
 * ═══════════════════════════════════════════════════════════════
 *
 *  Flash the SAME firmware on both boards.
 *
 *  Modes:
 *    TX     — send packets, wait for ACK, display RSSI/loss/RTT
 *    RX     — listen, ACK back, display RSSI/stats
 *    TRX    — ping-pong: both sides TX + RX alternately
 *    PER    — fixed 1000-packet test for industry-standard metric
 *    CW TX  — continuous rapid TX (max preamble) for antenna testing
 *    Sensitivity — auto power step-down test (when PA ctrl available)
 *    Reg Dump — display key CMT2310A register values
 *    View Logs — review past test sessions from EEPROM
 *
 *  Live enhancements:
 *    • RSSI bar graph on OLED
 *    • Link margin (RSSI − noise floor)
 *    • BER (Bit Error Rate) counter
 *    • Throughput (actual bytes/sec)
 *    • Rolling packet loss (last 32 packets)
 *    • Serial CSV output for plotting
 *    • EEPROM auto-logging per session
 *
 *  Buttons: UP/DOWN = navigate, LEFT/RIGHT = change, OK = start/stop
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "ar_types.h"       /* ARRes typedef — must be in a header so Arduino
                               auto-prototypes can resolve the type            */

struct Buttons;
struct RtcTime;
struct MeshRoute;
struct MeshBlacklist;
static void ar_serial_cmd();

/* ─── type aliases ─── */
typedef uint8_t  u8;
typedef uint16_t u16; 
typedef uint32_t u32;
typedef bool     BOOL;
#define TRUE   true
#define FALSE  false
#define INFINITE 0xFFFFFFFFUL

/* ─── TRX state machine ─── */
typedef enum {
  ST_IDLE = 0, ST_TX_START, ST_TX_STREAM, ST_TX_WAIT, ST_TX_DONE_ST, ST_TX_TIMEOUT,
  ST_RX_START, ST_RX_WAIT, ST_RX_DONE_ST, ST_ERROR
} TRXState;

typedef enum {
  RF_IDLE = 0, RF_BUSY, RF_TX_DONE, RF_RX_DONE,
  RF_TX_TIMEOUT, RF_RX_TIMEOUT, RF_ERROR
} RFResult;

/* ═══════════════════════════════════════════════════════════════
   ███ PIN MAP & ADDRESSES                                    ███
   ═══════════════════════════════════════════════════════════════ */
#define SDA_PIN   8
#define SCL_PIN   9
#define CMT_CSB   7
#define CMT_SCLK  4
#define CMT_MOSI  5
#define CMT_MISO  6
#define CMT_GPIO0 0   /* TX_DONE interrupt */
#define CMT_GPIO1 1   /* PKT_DONE interrupt */
#define ZB_TX     2
#define ZB_RX     3
#define PCA_INT   10

#define PCA_ADDR  0x20
#define LP_ADDR   0x2D
#define EE_ADDR   0x50
#define RTC_ADDR  0x51
#define OLED_ADDR 0x3C

/* ═══════════════════════════════════════════════════════════════
   ███ I2C HELPERS                                            ███
   ═══════════════════════════════════════════════════════════════ */
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
uint8_t i2cRead(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

/* ═══ WATCHDOG ═══ */
static uint8_t _port0 = (1 << 7);
void kick() { _port0 ^= (1 << 6); i2cWrite(PCA_ADDR, 0x02, _port0); }

/* ═══ RGB LED (non-blocking flash) ═══ */
static bool _ledOK = false;
static u32  _led_off_at = 0;        /* 0 = no pending auto-off */
void ledInit() {
  if (!i2cWrite(LP_ADDR, 0x00, 0x01)) return; delay(2);
  i2cWrite(LP_ADDR, 0x01, 0x00);
  i2cWrite(LP_ADDR, 0x14, 0xFF); i2cWrite(LP_ADDR, 0x15, 0xFF); i2cWrite(LP_ADDR, 0x16, 0xFF);
  i2cWrite(LP_ADDR, 0x02, 0x07); i2cWrite(LP_ADDR, 0x0F, 0x55);
  i2cWrite(LP_ADDR, 0x18, 0x00); i2cWrite(LP_ADDR, 0x19, 0x00); i2cWrite(LP_ADDR, 0x1A, 0x00);
  _ledOK = true;
}
void ledColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!_ledOK) return;
  i2cWrite(LP_ADDR, 0x19, r); i2cWrite(LP_ADDR, 0x1A, g); i2cWrite(LP_ADDR, 0x18, b);
}
void ledOff()   { _led_off_at = 0; ledColor(0,   0,   0);   }
void ledRed()   { ledColor(255, 0,   0);   }
void ledGreen() { ledColor(0,   255, 0);   }
void ledBlue()  { ledColor(0,   0,   255); }
void ledWhite() { ledColor(255, 255, 255); }
void ledYellow(){ ledColor(255, 200, 0);   }
/* Non-blocking: set color + auto-off after ms */
void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
  ledColor(r, g, b); _led_off_at = millis() + ms;
}
void ledFlashGreen(uint16_t ms) { ledFlash(0, 255, 0, ms); }
void ledFlashRed(uint16_t ms)   { ledFlash(255, 0, 0, ms); }
void ledFlashBlue(uint16_t ms)  { ledFlash(0, 0, 255, ms); }

/* ═══ BUTTONS (edge-detect, debounced, throttled I2C reads) ═══ */
struct Buttons { bool left, right, ok, up, down; uint8_t dip; };

/* Raw level read (2× I2C) — called max every BTN_POLL_MS */
static Buttons _rawButtons() {
  uint8_t p0 = i2cRead(PCA_ADDR, 0x00);
  uint8_t p1 = i2cRead(PCA_ADDR, 0x01);
  Buttons b; b.dip = p0 & 0x3F;
  b.left  = !(p1 & (1 << 4));
  b.right = !(p1 & (1 << 7));
  b.ok    = !(p1 & (1 << 6));
  b.up    = !(p1 & (1 << 5));
  b.down  = !(p1 & (1 << 3));
  return b;
}

#define BTN_POLL_MS   25    /* I2C read throttle */
#define BTN_DEBOUNCE  80    /* ms before same key re-fires */
static u32 _btn_poll_t = 0;
static u32 _btn_edge_t = 0;
static uint8_t _btn_prev = 0;     /* packed: bit0=L 1=R 2=OK 3=UP 4=DN */

static uint8_t _packBtn(const Buttons &b) {
  return (b.left) | (b.right << 1) | (b.ok << 2) | (b.up << 3) | (b.down << 4);
}

/* Returns edges only (press-down events). Throttles I2C to 25ms. */
Buttons readButtons() {
  Buttons out = {false, false, false, false, false, 0};
  u32 now = millis();
  if (now - _btn_poll_t < BTN_POLL_MS) return out;
  _btn_poll_t = now;

  Buttons raw = _rawButtons();
  out.dip = raw.dip;
  uint8_t cur = _packBtn(raw);
  uint8_t pressed = cur & ~_btn_prev;   /* newly pressed bits */
  _btn_prev = cur;

  if (pressed && (now - _btn_edge_t >= BTN_DEBOUNCE)) {
    _btn_edge_t = now;
    out.left  = pressed & 0x01;
    out.right = pressed & 0x02;
    out.ok    = pressed & 0x04;
    out.up    = pressed & 0x08;
    out.down  = pressed & 0x10;
  }
  return out;
}

/* ═══ EEPROM ═══ */
void eepromWrite(uint16_t addr, uint8_t val) {
  Wire.beginTransmission(EE_ADDR);
  Wire.write(addr >> 8); Wire.write(addr & 0xFF); Wire.write(val);
  Wire.endTransmission(); delay(5);
}
uint8_t eepromRead(uint16_t addr) {
  Wire.beginTransmission(EE_ADDR);
  Wire.write(addr >> 8); Wire.write(addr & 0xFF);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)EE_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
void eepromWriteU16(uint16_t addr, uint16_t val) {
  eepromWrite(addr, val >> 8); eepromWrite(addr + 1, val & 0xFF);
}
uint16_t eepromReadU16(uint16_t addr) {
  return ((uint16_t)eepromRead(addr) << 8) | eepromRead(addr + 1);
}
void eepromWriteU32(uint16_t addr, uint32_t val) {
  eepromWriteU16(addr, val >> 16); eepromWriteU16(addr + 2, val & 0xFFFF);
}
uint32_t eepromReadU32(uint16_t addr) {
  return ((uint32_t)eepromReadU16(addr) << 16) | eepromReadU16(addr + 2);
}

/* ═══ RTC ═══ */
struct RtcTime { uint8_t sec, min, hr, day, mon, yr; bool oscStopped; };
static uint8_t _dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
RtcTime rtcRead() {
  RtcTime t; uint8_t raw = i2cRead(RTC_ADDR, 0x04);
  t.oscStopped = (raw & 0x80) != 0; t.sec = raw & 0x7F;
  t.min = i2cRead(RTC_ADDR, 0x05) & 0x7F;
  t.hr  = i2cRead(RTC_ADDR, 0x06) & 0x3F;
  t.day = i2cRead(RTC_ADDR, 0x07) & 0x3F;
  t.mon = i2cRead(RTC_ADDR, 0x09) & 0x1F;
  t.yr  = i2cRead(RTC_ADDR, 0x0A);
  return t;
}
void rtcSetTime(uint8_t hr, uint8_t mn, uint8_t sc, uint8_t dy, uint8_t mo, uint8_t yr) {
  Wire.beginTransmission(RTC_ADDR); Wire.write(0x04);
  Wire.write(_dec2bcd(sc) & 0x7F); Wire.write(_dec2bcd(mn)); Wire.write(_dec2bcd(hr));
  Wire.write(_dec2bcd(dy)); Wire.write(0x00); Wire.write(_dec2bcd(mo));
  Wire.write(_dec2bcd(yr)); Wire.endTransmission();
}
void rtcInit() { RtcTime t = rtcRead(); if (t.oscStopped) rtcSetTime(12, 0, 0, 1, 1, 26); }

/* ═══════════════════════════════════════════════════════════════
   ███ OLED                                                   ███
   ═══════════════════════════════════════════════════════════════ */
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
static bool _oledOK = false;

void oledInit() {
  u8g2.setI2CAddress(OLED_ADDR << 1);
  _oledOK = u8g2.begin();
  if (!_oledOK) { Serial.println("[oled] failed"); return; }
  u8g2.clearBuffer(); u8g2.sendBuffer();
}

void oledStatus(const char *l1, const char *l2,
                const char *l3 = "", const char *l4 = "", const char *l5 = "") {
  if (!_oledOK) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 13, l1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 27, l2);
  if (l3[0]) u8g2.drawStr(0, 39, l3);
  if (l4[0]) u8g2.drawStr(0, 51, l4);
  if (l5[0]) u8g2.drawStr(0, 63, l5);
  u8g2.sendBuffer();
}

/* ═══════════════════════════════════════════════════════════════
   ███ APP STATE & GLOBAL VARIABLES                           ███
   ═══════════════════════════════════════════════════════════════ */

/* App modes */
enum AppMode {
  MODE_TX = 0, MODE_RX, MODE_TRX, MODE_PER, MODE_CW,
  MODE_REG_DUMP, MODE_VIEW_LOGS, MODE_AUTO_RANGE, MODE_COUNT
};
static const char * const mode_labels[] = {
  "TX", "RX", "TRX", "PER 1000", "CW TX", "RegDump", "Logs", "AutoRng"
};

/* App state machine */
enum AppState { APP_MENU, APP_RUNNING, APP_STOPPED, APP_CONN_LOST };
static AppState appState = APP_MENU;

/* Menu config */
static uint8_t  cfg_mode    = MODE_TX;
static uint8_t  cfg_rateIdx = 2;   /* default 2.4k@5kHz */
static uint8_t  cfg_sizeIdx = 2;   /* default 64 bytes */
static uint8_t  cfg_pwrIdx  = 1;   /* 0 = -10dBm, 1 = +20dBm (default max) */

/* ─── TX Power configuration ───
   CMT2310A PA registers on Page1 that differ between power levels.
   Patched at runtime after page1 bulk load.                        */
#define NUM_PWR_LEVELS 2
static const char * const pwr_labels[NUM_PWR_LEVELS] = { "-10", "+20" };
static const int8_t       pwr_dbm[NUM_PWR_LEVELS]    = { -10,   20   };

/* Page1 register patches: {idx30/0x9E, idx31/0x9F, idx33/0xA1, idx36/0xA4} */
static const uint8_t pwr_regs[NUM_PWR_LEVELS][4] = {
  /* -10 dBm */ { 0x18, 0x15, 0x4B, 0x01 },
  /* +20 dBm */ { 0x91, 0x7F, 0x18, 0x1F },
};

/* Statistics */
static u32 round_num     = 0;
static u32 ok_count      = 0;
static u32 fail_count    = 0;
static u32 corrupt_count = 0;
static u32 pktCount      = 0;   /* RX mode */
static u32 dup_count     = 0;   /* RX: retries detected via seq */
static u32 rx_err_count  = 0;   /* RX: packets received with bit errors */
static u32 sent_at       = 0;
static u32 g_last_rtt    = 0;   /* last round-trip time (ms) for TX OLED */
static u8  retries       = 0;

/* ─── Mesh addressing (CMT2310A hardware addr filter) ─── */
/* When enabled, radio-level address bytes go BEFORE the length field in FIFO:
 *   FIFO TX: [dest_addr] [src_addr] [length] [payload...]
 *   FIFO RX: [dest_addr] [src_addr] [length] [payload...]
 * The radio filters RX by dest_addr vs g_node_addr; broadcast = 0x00/0xFF.
 * These are separate from the app-level PKT_HDR dest/seq bytes.
 * Default OFF — enable with serial cmd "ADDR ON" after basic TX/RX is verified. */
static bool     g_addr_enabled  = false;  /* HW address filtering OFF by default */
static uint8_t  g_node_addr     = 0x01;   /* this node's radio address     */
static uint8_t  g_tx_dest_addr  = 0xFF;   /* TX target (0xFF=broadcast)    */
static uint8_t  g_rx_src_addr   = 0x00;   /* last RX: sender's address     */
static uint8_t  g_rx_dest_addr  = 0x00;   /* last RX: dest (us/broadcast)  */
static bool     g_addr_debug    = false;  /* verbose FIFO byte dumps       */
static bool     g_mesh_on       = false;  /* DIP6: multi-hop mesh relay        */

/* ═══════════════════════════════════════════════════════════════
   DIP SWITCH CONFIGURATION
   ═══════════════════════════════════════════════════════════════
   DIP switches are active-low (ON=0 in hardware).
   We invert so ON=1 in our logic.

   ┌─────┬──────────────┬───────────────────────────────────┐
   │ DIP │ Function     │ ON                  │ OFF         │
   ├─────┼──────────────┼─────────────────────┼─────────────┤
   │  1  │ Role         │ Board A (TX/Master) │ B (RX/Slave)│
   │  2  │ Addr filter  │ Mesh addressing ON  │ OFF (basic) │
   │  3  │ Addr bit 0 ──┐                                   │
   │  4  │ Addr bit 1  ─┤ Node address 0x01–0x08            │
   │  5  │ Addr bit 2 ──┘ (DIP 3/4/5 as 3-bit + 1)         │
   │  6  │ Mesh relay   │ Multi-hop relay ON  │ Direct only │
   └─────┴──────────────┴───────────────────────────────────┘

   Address table (DIP 3,4,5 — all OFF = 0x01):
     OFF OFF OFF = 0x01    ON  OFF OFF = 0x02
     OFF ON  OFF = 0x03    ON  ON  OFF = 0x04
     OFF OFF ON  = 0x05    ON  OFF ON  = 0x06
     OFF ON  ON  = 0x07    ON  ON  ON  = 0x08

   Common setups (DIP 1-6):
     Basic TX/RX :  A=[ON off off off off off]  B=[off off off off off off]
     2-node mesh :  A=[ON ON  off off off off]  B=[off ON  ON  off off off]
     3-node relay:  1=[ON ON  off off off ON ]  (TX, addr 0x01, relay)
                    2=[off ON  ON  off off ON ]  (RX/relay, addr 0x02)
                    3=[off ON  off ON  off ON ]  (RX/dest,  addr 0x03)
   ═══════════════════════════════════════════════════════════════ */
static void applyDIPConfig() {
  Buttons b = _rawButtons();
  uint8_t dip = (~b.dip) & 0x3F;   /* invert: ON=1 */

  /* DIP 1: Role (handled in startOperation / autoRangeRun, not here) */

  /* DIP 2: Address filtering */
  bool new_addr = (dip >> 1) & 1;

  /* DIP 3-5: Node address (3-bit + 1 → 0x01–0x08) */
  uint8_t addr_bits = (dip >> 2) & 0x07;
  uint8_t new_node  = addr_bits + 1;

  /* DIP 6: Mesh relay protocol */
  bool new_relay = (dip >> 5) & 1;
  if (new_relay) new_addr = true;   /* mesh relay requires addressing */

  g_addr_enabled = new_addr;
  g_node_addr    = new_node;
  g_mesh_on      = new_relay;
  if (g_mesh_on) g_tx_dest_addr = 0xFF;  /* broadcast until route learned */

  Serial.printf("[DIP] raw=0x%02X → role=%c  addr=%s  node=0x%02X  relay=%s\n",
                dip,
                (dip & 0x01) ? 'A' : 'B',
                g_addr_enabled ? "ON" : "off",
                g_node_addr,
                g_mesh_on ? "ON" : "off");
}

/* ═══════════════════════════════════════════════════════════════
   MESH RELAY PROTOCOL — Multi-hop forwarding with route learning
   ═══════════════════════════════════════════════════════════════

   Packet format (inside FIFO, after radio addr bytes):

     Radio:  [next_hop][my_addr][fifo_len]  ← CMT2310A HW address filter
     Mesh:   [m_dest][m_src][hops][ttl][mseq]  ← MESH_HDR (5 B)
     App:    [dest_id][seq][data...]             ← PKT_HDR + payload

   The CMT2310A "Detect Address 0x00 0xFF" mode accepts in HW:
     dest == my_addr  OR  dest == 0x00  OR  dest == 0xFF
   Everything else is dropped at the radio — zero CPU cost.

   Relay decision (software):
     mesh_dest == my_addr → deliver to app, send ACK back
     mesh_dest != my_addr, ttl > 0, not seen → relay (forward)
     mesh_dest == 0xFF (broadcast) → process + relay (no ACK)

   Route learning (passive — every received packet):
     Learn route to mesh_src via radio_src (last hop)
   ═══════════════════════════════════════════════════════════════ */

#define MESH_HDR          5      /* m_dest(1) + m_src(1) + hops(1) + ttl(1) + mseq(1) */
#define MESH_MAX_TTL      5      /* max relay hops before drop */
#define MESH_ROUTE_MAX    8      /* route table entries */
#define MESH_SEEN_MAX     32     /* dedup ring buffer entries */
#define MESH_RELAY_DELAY  15     /* ms delay before relay TX (collision avoidance) */
#define MESH_ROUTE_EXPIRY   30000  /* ms — stale routes expire after 30s no refresh   */
#define MESH_FAIL_LIMIT     3      /* consecutive ACK fails → invalidate route         */
#define MESH_BLACKLIST_MAX  8      /* max blacklisted next-hops                        */
#define MESH_BLACKLIST_TIME 120000 /* ms — blacklist a dead next-hop for 2 minutes     */
#define MESH_MAX_NODE_ADDR  0x08   /* highest valid node address (DIP 3-bit + 1)       */

/* Validate mesh header — reject corrupted packets before learning garbage routes */
static bool mesh_hdr_valid(uint8_t m_dest, uint8_t m_src, uint8_t m_hops, uint8_t m_ttl) {
  if (m_src == 0x00 || m_src == 0xFF) return false;    /* src can't be broadcast */
  if (m_src > MESH_MAX_NODE_ADDR) return false;         /* src out of range       */
  if (m_dest != 0xFF && m_dest != 0x00 &&
      m_dest > MESH_MAX_NODE_ADDR) return false;         /* dest out of range      */
  if (m_hops >= MESH_MAX_TTL) return false;              /* hops can't exceed TTL  */
  if (m_ttl > MESH_MAX_TTL) return false;                /* TTL sanity             */
  return true;
}

static uint8_t g_mesh_final_dest = 0xFF; /* mesh-level final destination */
static uint8_t g_mesh_tx_seq     = 0;    /* outgoing mesh sequence counter */
static uint8_t g_mesh_fail_cnt   = 0;    /* consecutive ACK failures on current route */

/* ─── Next-hop blacklist ───
   When a relay node fails, we blacklist its address so passive route learning
   doesn't re-add a route through the dead node.  Blacklist expires after
   MESH_BLACKLIST_TIME, giving a repaired node a second chance.               */
struct MeshBlacklist {
  uint8_t  addr;       /* blacklisted next-hop address */
  uint32_t until;      /* millis() when blacklist expires */
  bool     active;
};
static MeshBlacklist g_blacklist[MESH_BLACKLIST_MAX];

static bool mesh_is_blacklisted(uint8_t next_hop) {
  for (int i = 0; i < MESH_BLACKLIST_MAX; i++) {
    if (!g_blacklist[i].active || g_blacklist[i].addr != next_hop) continue;
    if (millis() > g_blacklist[i].until) {
      /* Expired — give this node another chance */
      g_blacklist[i].active = false;
      Serial.printf("[MESH] Blacklist expired for 0x%02X — node can be used again\n", next_hop);
      return false;
    }
    return true;  /* still blacklisted */
  }
  return false;
}

static void mesh_blacklist_add(uint8_t next_hop) {
  /* Already blacklisted? refresh timer */
  for (int i = 0; i < MESH_BLACKLIST_MAX; i++) {
    if (g_blacklist[i].active && g_blacklist[i].addr == next_hop) {
      g_blacklist[i].until = millis() + MESH_BLACKLIST_TIME;
      return;
    }
  }
  /* Find empty slot or overwrite oldest */
  int best = 0; uint32_t earliest = 0xFFFFFFFF;
  for (int i = 0; i < MESH_BLACKLIST_MAX; i++) {
    if (!g_blacklist[i].active) { best = i; break; }
    if (g_blacklist[i].until < earliest) { earliest = g_blacklist[i].until; best = i; }
  }
  g_blacklist[best].addr   = next_hop;
  g_blacklist[best].until  = millis() + MESH_BLACKLIST_TIME;
  g_blacklist[best].active = true;
  Serial.printf("[MESH] Blacklisted next-hop 0x%02X for %us (dead relay)\n",
                next_hop, MESH_BLACKLIST_TIME / 1000);
}

static void mesh_blacklist_clear() {
  for (int i = 0; i < MESH_BLACKLIST_MAX; i++) g_blacklist[i].active = false;
}

/* ─── Route table (AODV-lite) ─── */
struct MeshRoute {
  uint8_t  dest;       /* final destination node */
  uint8_t  next_hop;   /* radio-level next hop to reach dest */
  uint8_t  hops;       /* hop count */
  int16_t  rssi;       /* best RSSI seen on this route (link quality) */
  uint32_t ts;         /* millis() at last update / refresh */
  bool     valid;
};
static MeshRoute g_routes[MESH_ROUTE_MAX];

/* Find route — returns NULL if missing, expired, OR next-hop blacklisted */
static MeshRoute* mesh_route_find(uint8_t dest) {
  for (int i = 0; i < MESH_ROUTE_MAX; i++) {
    if (!g_routes[i].valid || g_routes[i].dest != dest) continue;
    /* Expire stale routes */
    if (millis() - g_routes[i].ts > MESH_ROUTE_EXPIRY) {
      g_routes[i].valid = false;
      Serial.printf("[MESH] Route to 0x%02X expired (stale %lus)\n",
                    dest, (unsigned long)(MESH_ROUTE_EXPIRY / 1000));
      return NULL;
    }
    /* Skip routes through blacklisted (dead) relays */
    if (mesh_is_blacklisted(g_routes[i].next_hop)) {
      Serial.printf("[MESH] Route to 0x%02X via 0x%02X SKIPPED (next-hop blacklisted)\n",
                    dest, g_routes[i].next_hop);
      return NULL;  /* force broadcast, don't invalidate — route may revive */
    }
    return &g_routes[i];
  }
  return NULL;
}

/* Update / learn route.  Rejects routes through blacklisted relays.
   Prefers: fewer hops → better RSSI → fresher */
static void mesh_route_update(uint8_t dest, uint8_t via, uint8_t hops, int16_t rssi){
  if (dest == g_node_addr) return;
  if (dest == 0xFF || dest == 0x00) return;
  /* Don't learn routes through dead relays */
  if (mesh_is_blacklisted(via)) return;

  MeshRoute *r = mesh_route_find(dest);
  if (r) {
    bool better = (hops < r->hops) ||
                  (hops == r->hops && rssi > r->rssi);
    if (better || via == r->next_hop) {
      /* Better route, OR refresh existing route */
      if (better && (r->next_hop != via || r->hops != hops)) {
        Serial.printf("[MESH] Route improved: 0x%02X via 0x%02X %uhop RSSI=%d (was via 0x%02X %uhop RSSI=%d)\n",
                      dest, via, hops, rssi, r->next_hop, r->hops, r->rssi);
      }
      r->next_hop = via; r->hops = hops; r->rssi = rssi; r->ts = millis();
    }
    if(!better) Serial.printf("[mesh_route_update] Route for 0x%02X is already there\n",dest);
    return;
  }
  /* New route — find empty slot or evict worst (most hops, then oldest) */
  int best = 0; uint8_t worst_hops = 0; uint32_t oldest = 0xFFFFFFFF;
  for (int i = 0; i < MESH_ROUTE_MAX; i++) {
    if (!g_routes[i].valid) { best = i; break; }
    if (g_routes[i].hops > worst_hops ||
        (g_routes[i].hops == worst_hops && g_routes[i].ts < oldest)) {
      worst_hops = g_routes[i].hops; oldest = g_routes[i].ts; best = i;
    }
  }
  g_routes[best].dest = dest;     g_routes[best].next_hop = via;
  g_routes[best].hops = hops;     g_routes[best].rssi = rssi;
  g_routes[best].ts   = millis(); g_routes[best].valid = true;
  Serial.printf("[MESH] Route learned: 0x%02X via 0x%02X (%u hop%s, RSSI=%d)\n",
                dest, via, hops, hops == 1 ? "" : "s", rssi);
}

/* Invalidate a specific route AND blacklist its next-hop */
static void mesh_route_invalidate(uint8_t dest) {
  for (int i = 0; i < MESH_ROUTE_MAX; i++) {
    if (g_routes[i].valid && g_routes[i].dest == dest) {
      uint8_t dead_hop = g_routes[i].next_hop;
      g_routes[i].valid = false;
      mesh_blacklist_add(dead_hop);  /* prevent re-learning through dead relay */
      Serial.printf("[MESH] Route to 0x%02X via 0x%02X INVALIDATED + blacklisted\n",
                    dest, dead_hop);
    }
  }
}

static void mesh_route_clear() {
  for (int i = 0; i < MESH_ROUTE_MAX; i++) g_routes[i].valid = false;
  mesh_blacklist_clear();
  Serial.println("[MESH] All routes + blacklist cleared");
}

static void mesh_route_print() {
  Serial.println("\n=== Mesh Route Table ===");
  bool any = false;
  u32 expiry_s = MESH_ROUTE_EXPIRY / 1000;
  for (int i = 0; i < MESH_ROUTE_MAX; i++) {
    if (!g_routes[i].valid) continue;
    u32 age_ms = millis() - g_routes[i].ts;
    u32 age_s  = age_ms / 1000;
    /* Clean up expired entries while printing */
    if (age_ms > MESH_ROUTE_EXPIRY) {
      g_routes[i].valid = false;
      Serial.printf("  dest=0x%02X  via=0x%02X  *** EXPIRED (age %lus)\n",
                    g_routes[i].dest, g_routes[i].next_hop, (unsigned long)age_s);
      continue;
    }
    any = true;
    MeshRoute &r = g_routes[i];
    int32_t remaining = (int32_t)expiry_s - (int32_t)age_s;
    if (remaining < 0) remaining = 0;
    bool bl = mesh_is_blacklisted(r.next_hop);
    Serial.printf("  dest=0x%02X  via=0x%02X  hops=%u  RSSI=%d  age=%lus  expires=%lds%s\n",
                  r.dest, r.next_hop, r.hops, r.rssi,
                  (unsigned long)age_s, (long)remaining,
                  bl ? "  *** BLOCKED (dead relay)" : "");
  }
  if (!any) Serial.println("  (empty — routes learned after first packet exchange)");
  Serial.printf("  Fail counter: %u/%u\n", g_mesh_fail_cnt, MESH_FAIL_LIMIT);

  /* Print blacklist */
  bool anyBL = false;
  for (int i = 0; i < MESH_BLACKLIST_MAX; i++) {
    if (!g_blacklist[i].active) continue;
    if (!anyBL) { Serial.println("  --- Blacklisted Relays ---"); anyBL = true; }
    u32 rem = 0;
    if (millis() < g_blacklist[i].until) rem = (g_blacklist[i].until - millis()) / 1000;
    Serial.printf("  0x%02X  unblocks in %lus\n", g_blacklist[i].addr, (unsigned long)rem);
  }
  Serial.println("========================\n");
}

/* ─── Dedup ring buffer ─── */
static struct { uint8_t src; uint8_t seq; } g_seen[MESH_SEEN_MAX];
static uint8_t g_seen_wr = 0;

static bool mesh_is_seen(uint8_t src, uint8_t seq) {
  for (int i = 0; i < MESH_SEEN_MAX; i++)
    if (g_seen[i].src == src && g_seen[i].seq == seq) return true;
  return false;
}
static void mesh_mark_seen(uint8_t src, uint8_t seq) {
  g_seen[g_seen_wr].src = src; g_seen[g_seen_wr].seq = seq;
  g_seen_wr = (g_seen_wr + 1) % MESH_SEEN_MAX;
}

/* ─── Relay buffer ─── */
static uint8_t  g_relay_buf[600];   /* relay packet copy (> MAX_PAYLOAD) */
static uint16_t g_relay_len = 0;
static bool     g_relay_pending = false;

/* Set radio-level dest from route table (or broadcast if no route).
   Returns true if route was found, false if broadcasting. */
static bool mesh_set_radio_dest(uint8_t mesh_dest) {
  MeshRoute *r = mesh_route_find(mesh_dest);
  if (r) {
    g_tx_dest_addr = r->next_hop;
    Serial.printf("[MESH] route 0x%02X → hop=0x%02X (%uhop RSSI=%d)\n",
                  mesh_dest, r->next_hop, r->hops, r->rssi);
    return true;
  }
  g_tx_dest_addr = 0xFF;  /* no route — broadcast discovery */
  if (mesh_dest != 0xFF)
    Serial.printf("[MESH] no route to 0x%02X — broadcast discovery\n", mesh_dest);
  return false;
}

/* ─── Stop-and-Wait ARQ: sequence numbers ─── */
/* Packet format: [dest 1B] [seq 1B] [data NB]
 *   → payload = 2 + data_len
 *   → TX sends with current tx_seq; retries keep same seq
 *   → RX checks seq: if == last_seen → dup (ACK but don't count)
 *   → ACK echoes seq so TX can verify it matches               */
#define PKT_HDR 2                /* dest + seq bytes before data */
static u8  tx_seq       = 0;    /* TX: increments on new packet */
static u8  rx_last_seq  = 0xFF; /* RX: last seq received (init impossible) */

/* RSSI tracking */
int16_t g_last_rssi       = -128;
static int16_t rssi_best  = -128;
static int16_t rssi_worst = 127;
static int32_t rssi_sum   = 0;
static u32     rssi_cnt   = 0;

/* BER tracking */
static u32 total_bits     = 0;
static u32 error_bits     = 0;

/* Throughput tracking */
static u32 throughput_bytes = 0;
static u32 throughput_start = 0;

/* Link margin — noise floor measured from chip */
#define NOISE_FLOOR_DBM (-117)

/* Rolling packet loss — last 32 packets */
#define LOSS_WINDOW 32
static u32 loss_ring = 0;          /* bitfield: 1=ok, 0=fail */
static u8  loss_ring_cnt = 0;      /* how many bits valid */

/* RSSI history for bar graph — last 16 readings */
#define RSSI_HIST_LEN 16
static int16_t rssi_hist[RSSI_HIST_LEN];
static u8 rssi_hist_idx = 0;
static u8 rssi_hist_cnt = 0;

/* OLED refresh limiter */
static u32 last_oled_ms = 0;
#define OLED_INTERVAL 150

/* Connection-lost detection */
#define MAX_CONSEC_FAIL 5
static u8 consec_fail_rounds = 0;
static u32 rx_last_pkt_ms   = 0;
#define RX_CONN_LOST_MS  30000

/* PER test */
#define PER_COUNT 1000
static u32 per_target = PER_COUNT;

/* EEPROM log — each entry = 16 bytes, starts at addr 0x0100, max 30 entries */
#define LOG_BASE_ADDR  0x0100
#define LOG_ENTRY_SIZE 16
#define LOG_MAX_ENTRIES 30
#define LOG_COUNT_ADDR 0x00F0   /* 2 bytes: count of valid entries */

static void resetStats() {
  round_num = ok_count = fail_count = corrupt_count = pktCount = dup_count = rx_err_count = 0;
  sent_at = 0; retries = 0; g_last_rtt = 0;
  tx_seq = 0; rx_last_seq = 0xFF;
  rssi_best = -128; rssi_worst = 127; rssi_sum = 0; rssi_cnt = 0;
  g_last_rssi = -128; consec_fail_rounds = 0;
  last_oled_ms = 0; rx_last_pkt_ms = 0;
  total_bits = error_bits = 0;
  throughput_bytes = 0; throughput_start = millis();
  loss_ring = 0; loss_ring_cnt = 0;
  rssi_hist_idx = 0; rssi_hist_cnt = 0;
  memset(rssi_hist, 0, sizeof(rssi_hist));
}

/* ═══════════════════════════════════════════════════════════════
   ███ HELPER FUNCTIONS                                       ███
   ═══════════════════════════════════════════════════════════════ */
void feedLoop() {
  static unsigned long _lk = 0;
  u32 now = millis();
  if (now - _lk >= 300) { kick(); _lk = now; }
  if (_led_off_at && now >= _led_off_at) { _led_off_at = 0; ledColor(0, 0, 0); }
  ar_serial_cmd();
}
/* Legacy blocking flash — only used in splash/init, NOT in loops */
void flashGreen(uint16_t ms) {
  ledGreen(); u32 t0 = millis();
  while (millis() - t0 < ms) { feedLoop(); delay(5); }
  ledOff();
}

const char* rssiQ(int16_t r) {
  if (r > -50)  return "Excellent";
  if (r > -70)  return "Good";
  if (r > -85)  return "Fair";
  if (r > -100) return "Weak";
  return "V.Weak";
}

static void updateRSSIStats(int16_t r) {
  if (r > rssi_best)  rssi_best  = r;
  if (r < rssi_worst) rssi_worst = r;
  rssi_sum += r;
  rssi_cnt++;
  /* Update history ring for bar graph */
  rssi_hist[rssi_hist_idx] = r;
  rssi_hist_idx = (rssi_hist_idx + 1) % RSSI_HIST_LEN;
  if (rssi_hist_cnt < RSSI_HIST_LEN) rssi_hist_cnt++;
}

static void lossRingPush(bool success) {
  loss_ring = (loss_ring << 1) | (success ? 1 : 0);
  if (loss_ring_cnt < LOSS_WINDOW) loss_ring_cnt++;
}

static float lossRingPercent() {
  if (loss_ring_cnt == 0) return 0.0f;
  u8 ok = 0;
  u32 mask = loss_ring;
  for (u8 i = 0; i < loss_ring_cnt; i++) {
    if (mask & 1) ok++;
    mask >>= 1;
  }
  return 100.0f * (float)(loss_ring_cnt - ok) / (float)loss_ring_cnt;
}

/* BER: count mismatched bits between two buffers */
static u32 countBitErrors(const u8 *a, const u8 *b, u16 len) {
  u32 errs = 0;
  for (u16 i = 0; i < len; i++) {
    u8 x = a[i] ^ b[i];
    while (x) { errs += (x & 1); x >>= 1; }
  }
  return errs;
}

/* Throughput in bytes/sec */
static u32 getThroughputBps() {
  u32 elapsed = millis() - throughput_start;
  if (elapsed == 0) return 0;
  return (throughput_bytes * 1000UL) / elapsed;
}

static void print_hex(const u8 *d, u8 len) {
  for (u8 i = 0; i < len; i++) Serial.printf("%02X ", d[i]);
  Serial.println();
}

/* Serial CSV header */
static void printCSVHeader() {
  Serial.println("timestamp_ms,pkt_num,rssi_dBm,loss_pct,rtt_ms,ber,throughput_Bps,margin_dB,temp");
}

/* Serial CSV line */
static void printCSVLine(u32 rtt) {
  int16_t margin = g_last_rssi - NOISE_FLOOR_DBM;
  u8 temp = 0;  /* will read from chip */
  float ber_f = (total_bits > 0) ? (float)error_bits / (float)total_bits : 0.0f;
  Serial.printf("%lu,%lu,%d,%.1f,%lu,%.6f,%lu,%d,%u\n",
    (unsigned long)millis(),
    (unsigned long)(round_num > 0 ? round_num : pktCount),
    (int)g_last_rssi,
    lossRingPercent(),
    (unsigned long)rtt,
    ber_f,
    (unsigned long)getThroughputBps(),
    (int)margin,
    temp);
}

/* ═══════════════════════════════════════════════════════════════
   ███ OLED DRAW HELPERS                                      ███
   ═══════════════════════════════════════════════════════════════ */

/* Draw RSSI bar graph — bottom 10px of screen, full width */
static void drawRSSIBar(int16_t rssi) {
  /* Map RSSI from [-120..-10] to [0..118] pixels */
  int w = (int)(rssi + 120);
  if (w < 0) w = 0;
  if (w > 118) w = 118;
  /* Bar background */
  u8g2.drawFrame(4, 54, 120, 10);
  /* Filled bar */
  if (w > 0) u8g2.drawBox(5, 55, w, 8);
  /* Marker labels */
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(0, 53, "-120");
  u8g2.drawStr(106, 53, "-10");
}

/* Draw RSSI mini history — 16 bars, each 4px wide */
static void drawRSSIHistory(int x, int y, int h) {
  for (u8 i = 0; i < rssi_hist_cnt; i++) {
    int idx = (rssi_hist_idx - rssi_hist_cnt + i + RSSI_HIST_LEN) % RSSI_HIST_LEN;
    int val = (int)(rssi_hist[idx] + 120);
    if (val < 0) val = 0; if (val > 110) val = 110;
    int bh = (val * h) / 110;
    if (bh < 1) bh = 1;
    u8g2.drawBox(x + i * 4, y + h - bh, 3, bh);
  }
}

/* ─── Splash / Boot Screen ─── */
static void drawSplash() {
  if (!_oledOK) return;
  u8g2.clearBuffer();

  /* RF antenna icon */
  u8g2.drawLine(64, 4, 64, 22);
  u8g2.drawDisc(64, 3, 2);
  u8g2.drawTriangle(58, 24, 70, 24, 64, 20);
  /* RF waves */
  u8g2.drawCircle(64, 12, 8,  U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_LOWER_LEFT);
  u8g2.drawCircle(64, 12, 14, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_LOWER_LEFT);
  u8g2.drawCircle(64, 12, 20, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_LOWER_LEFT);
  u8g2.drawCircle(64, 12, 8,  U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_LOWER_RIGHT);
  u8g2.drawCircle(64, 12, 14, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_LOWER_RIGHT);
  u8g2.drawCircle(64, 12, 20, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_LOWER_RIGHT);

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(4, 42, "Zb-Rf Board V1.0");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(36, 56, "by lohith D");
  u8g2.sendBuffer();
}

/* ─── Connection-Lost Cat Screen ─── */
static void drawCatLost(int16_t lastRSSI) {
  if (!_oledOK) return;
  u8g2.clearBuffer();

  int cx = 24, cy = 22;
  u8g2.drawCircle(cx, cy, 14);
  /* Ears */
  u8g2.drawTriangle(cx-14, cy-6, cx-10, cy-18, cx-4, cy-12);
  u8g2.drawTriangle(cx+14, cy-6, cx+10, cy-18, cx+4, cy-12);
  /* X eyes */
  u8g2.drawLine(cx-6, cy-4, cx-2, cy); u8g2.drawLine(cx-6, cy, cx-2, cy-4);
  u8g2.drawLine(cx+2, cy-4, cx+6, cy); u8g2.drawLine(cx+2, cy, cx+6, cy-4);
  /* Nose & mouth */
  u8g2.drawTriangle(cx-1, cy+4, cx+1, cy+4, cx, cy+6);
  u8g2.drawCircle(cx, cy+14, 5, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  /* Whiskers */
  u8g2.drawLine(cx-14, cy+2, cx-6, cy+4); u8g2.drawLine(cx-14, cy+6, cx-6, cy+6);
  u8g2.drawLine(cx+6, cy+4, cx+14, cy+2); u8g2.drawLine(cx+6, cy+6, cx+14, cy+6);

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(48, 13, "Connect");
  u8g2.drawStr(48, 27, "Lost!");
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];
  snprintf(buf, sizeof(buf), "RSSI:%d dBm", (int)lastRSSI);
  u8g2.drawStr(48, 42, buf);
  u8g2.drawStr(48, 54, "=Range Limit=");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(28, 63, "Press OK for menu");
  u8g2.sendBuffer();
}

/* ─── Stopped / Stats Screen ─── */
static void drawStoppedScreen() {
  if (!_oledOK) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 13, "-- STOPPED --");
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];
  u32 total = round_num > 0 ? round_num : pktCount;
  u32 good  = ok_count  > 0 ? ok_count  : pktCount;
  snprintf(buf, sizeof(buf), "Pkts:%lu Ok:%lu", (unsigned long)total, (unsigned long)good);
  u8g2.drawStr(0, 27, buf);
  snprintf(buf, sizeof(buf), "RSSI:%d dBm", (int)g_last_rssi);
  u8g2.drawStr(0, 39, buf);
  if (rssi_cnt > 0) {
    int16_t avg = (int16_t)(rssi_sum / (int32_t)rssi_cnt);
    snprintf(buf, sizeof(buf), "B:%d W:%d A:%d", (int)rssi_best, (int)rssi_worst, (int)avg);
  } else if (round_num > 0) {
    float loss = 100.0f * (float)(fail_count + corrupt_count) / (float)round_num;
    snprintf(buf, sizeof(buf), "Loss:%.1f%% BER:%.1e", loss,
             total_bits > 0 ? (float)error_bits/(float)total_bits : 0.0f);
  } else {
    snprintf(buf, sizeof(buf), "No data");
  }
  u8g2.drawStr(0, 51, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(14, 63, "Press OK for menu");
  u8g2.sendBuffer();
}

/* ═══ BOARD INIT ═══ */
void boardInit() {
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); Wire.setTimeout(1000);
  i2cWrite(PCA_ADDR, 0x06, 0b00111111);
  i2cWrite(PCA_ADDR, 0x02, _port0); kick();
  Serial.begin(115200);
  for (int i = 0; i < 10; i++) { delay(200); kick(); }
  i2cWrite(PCA_ADDR, 0x07, 0b11111000);
  i2cWrite(PCA_ADDR, 0x03, 0x00); kick();
  ledInit(); kick(); oledInit(); kick(); rtcInit(); kick();
  pinMode(CMT_GPIO0, INPUT);
  pinMode(CMT_GPIO1, INPUT);
}

/* ═══════════════════════════════════════════════════════════════
   ███ CMT2310A REGISTER DEFINES                              ███
   ═══════════════════════════════════════════════════════════════ */
#define CMT2310A_CTL_REG_00  0x00
#define CMT2310A_CTL_REG_01  0x01
#define CMT2310A_CTL_REG_04  0x04
#define CMT2310A_CTL_REG_07  0x07
#define CMT2310A_CTL_REG_08  0x08
#define CMT2310A_CTL_REG_09  0x09
#define CMT2310A_CTL_REG_10  0x0A
#define CMT2310A_CTL_REG_13  0x0D
#define CMT2310A_CTL_REG_14  0x0E
#define CMT2310A_CTL_REG_16  0x10
#define CMT2310A_CTL_REG_17  0x11
#define CMT2310A_CTL_REG_18  0x12
#define CMT2310A_CTL_REG_19  0x13
#define CMT2310A_CTL_REG_20  0x14
#define CMT2310A_CTL_REG_22  0x16
#define CMT2310A_CTL_REG_24  0x18
#define CMT2310A_CTL_REG_25  0x19
#define CMT2310A_CTL_REG_26  0x1A
#define CMT2310A_CTL_REG_27  0x1B
#define CMT2310A_CTL_REG_28  0x1C
#define CMT2310A_CTL_REG_33  0x21   /* RSSI_VALUE_MIN (antenna diversity) */
#define CMT2310A_CTL_REG_34  0x22   /* RSSI_VALUE — dBm, signed (AN236) */
#define CMT2310A_CTL_REG_35  0x23   /* LBD_DATA — low battery detection */
#define CMT2310A_CTL_REG_36  0x24   /* TEMP_DATA — temperature sensor */
#define CMT2310A_CTL_REG_61  0x3D
#define CMT2310A_CTL_REG_62  0x3E
#define CMT2310A_CTL_REG_63  0x3F
#define CMT2310A_CTL_REG_126 0x7E
#define CMT2310A_SOFT_RST    0x7F
#define CMT2310A_FIFO_PORT   0x7A
#define CMT2310A_CRW_PORT    0x7B

#define VAL_BIT0 0x01
#define VAL_BIT1 0x02
#define VAL_BIT2 0x04
#define VAL_BIT3 0x08
#define VAL_BIT4 0x10
#define VAL_BIT5 0x20
#define VAL_BIT6 0x40
#define VAL_BIT7 0x80
#define CMT_BIT6 6

#define M_GO_RX        VAL_BIT3
#define M_GO_TX        VAL_BIT2
#define M_GO_READY     VAL_BIT1
#define M_GO_SLEEP     VAL_BIT0
#define M_GPIO0_SEL    (VAL_BIT2 | VAL_BIT1 | VAL_BIT0)
#define M_GPIO1_SEL    (VAL_BIT5 | VAL_BIT4 | VAL_BIT3)
#define GPIO0_SEL_INT1 0x01
#define GPIO1_SEL_INT2 0x02
#define CMT2310A_CTL_REG_07_MASK (VAL_BIT7 | VAL_BIT6)
#define M_API_CMD_FLAG VAL_BIT7
#define M_INT1_SEL     (VAL_BIT5|VAL_BIT4|VAL_BIT3|VAL_BIT2|VAL_BIT1|VAL_BIT0)
#define M_INT2_SEL     (VAL_BIT5|VAL_BIT4|VAL_BIT3|VAL_BIT2|VAL_BIT1|VAL_BIT0)
#define M_INT1_POLAR   VAL_BIT7
#define M_INT2_POLAR   VAL_BIT6
#define INT_SRC_TX_DONE  0x10
#define INT_SRC_PKT_DONE 0x08
#define M_TX_DONE_EN       VAL_BIT5
#define M_PREAM_PASS_EN    VAL_BIT4
#define M_SYNC_PASS_EN     VAL_BIT3
#define M_ADDR_PASS_EN     VAL_BIT2
#define M_CRC_PASS_EN      VAL_BIT1
#define M_PKT_DONE_EN      VAL_BIT0
#define M_RX_FIFO_FULL_RX_EN  VAL_BIT7
#define M_RX_FIFO_NMTY_RX_EN  VAL_BIT6
#define M_RX_FIFO_TH_RX_EN    VAL_BIT5
#define M_RX_FIFO_OVF_EN      VAL_BIT3
#define M_TX_FIFO_FULL_EN     VAL_BIT2
#define M_TX_FIFO_NMTY_EN     VAL_BIT1
#define M_TX_FIFO_TH_EN       VAL_BIT0
#define M_PA_DIFF_SEL      VAL_BIT3
#define M_TX_DONE_FLG      VAL_BIT3
#define M_TX_DONE_CLR      VAL_BIT0
#define M_PREAM_PASS_CLR   VAL_BIT4   /* CTL_REG_25 bit4 */
#define M_SYNC_PASS_CLR    VAL_BIT3   /* CTL_REG_25 bit3 */
#define M_ADDR_PASS_CLR    VAL_BIT2   /* CTL_REG_25 bit2 */
#define M_CRC_PASS_CLR     VAL_BIT1   /* CTL_REG_25 bit1 */
#define M_PKT_DONE_CLR     VAL_BIT0   /* CTL_REG_25 bit0 */
#define M_PKT_DONE_FLG     VAL_BIT0   /* CTL_REG_26 bit0 */
#define M_TX_FIFO_CLR      VAL_BIT0
#define M_RX_FIFO_CLR      VAL_BIT1
#define M_TX_FIFO_TH_FLG   VAL_BIT0   /* CTL_REG_28 bit0: 1 = unsent > TH */
#define M_TX_FIFO_NMTY_FLG VAL_BIT1   /* CTL_REG_28 bit1: 1 = FIFO not empty */
#define M_RX_FIFO_TH_FLG   VAL_BIT5   /* CTL_REG_28 bit5: 1 = unread > TH */
#define M_RX_FIFO_OVF_FLG  VAL_BIT3   /* CTL_REG_28 bit3: 1 = RX FIFO overflow */
#define M_LENGTH_SIZE       VAL_BIT5   /* CTL_REG_63 bit5: 0=1-byte, 1=2-byte */
#define M_PKT_TYPE         VAL_BIT0
#define M_FIFO_MERGE_EN    VAL_BIT1
#define M_FIFO_TX_RX_SEL   VAL_BIT0
#define M_FIFO_AUTO_CLR_RX VAL_BIT4
#define M_HV_PAGE_SEL      (VAL_BIT7 | VAL_BIT6)
#define STATE_IS_READY 0x82
#define STATE_IS_RX    0x90
#define STATE_IS_TX    0xA0

/* FIFO streaming constants — merged FIFO (256 bytes shared TX+RX) */
#define FIFO_SIZE      256
#define FIFO_TH_VAL    200            /* high threshold: avoids streaming for ≤199-byte packets */

/* ── Deviation-sweep mode ──────────────────────────────────────
 * Uncomment DEV_SWEEP to replace the 4 rate slots with
 * 4 deviation variants (0.6 / 2.4 / 3.6 / 5.0 kHz) all at
 * 1.2 kbps 865 MHz.  Flash BOTH boards, run autorange at
 * close range (~5-10 m), then compare BER per config.
 * Comment out again and re-flash to go back to normal.       */
// #define DEV_SWEEP   /* done — 2.4 kHz chosen for 1.2 kbps */

#define CMT2310A_PAGE0_SIZE (0x77 - 0x28 + 1)   /* 80 bytes */
#define CMT2310A_PAGE1_SIZE (0xEF - 0x80 + 1)   /* 112 bytes */

/* ═══════════════════════════════════════════════════════════════
   ███ RFPDK-GENERATED REGISTER ARRAYS                        ███
   ═══════════════════════════════════════════════════════════════
   CMOSTEK RFPDK_V1.66   865 MHz FSK  TX +20 dBm
   1.2k: Dev 2.4/3.6 kHz  |  2.4k: Dev 5 kHz  |  9.6k/19.2k: Dev 10 kHz  |  PJD+RSSI
   Preamble TX=32B, 8-Jump window, RSSI_UPDATE_SEL=SYNC_OK
   ═══════════════════════════════════════════════════════════════ */

/* Page 0 — identical across all data rates */
static const uint8_t g_page0[CMT2310A_PAGE0_SIZE] = {
    0x12,0x20,0x00,0xAA,0x04,0x00,0x00,0x00,0x00,0x00,0xD5,0xD4, /* [1]=0x20: Preamble TX=32B */
    0x2D,0x00,0x00,0x00,0x00,0x00,0xD5,0xD4,0x2D,0x1F,0x00,0x00,
    0x00,0x00,0x00,0x00,0x2D,0x00,0x00,0x00,0x2D,0x00,0x00,0x00,
    0x00,0x00,0x45,0x1F,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,
    0x00,0x1F,0x00,0x00,0x00,0x00,0x00,0xE4,0x12,0x10,0x20,0x01, /* [50]=RSSI_CAL_OFFSET=0 */
    0x00,0xD0,0xE0,0xE2,0x84,0x30,0x08,0xD0,0xE0,0x6B,0x00,0x41,
    0x00,0x01,0x00,0x02,0x00,0x00,0x03,0x04,
};

/* ─── Mutable page0 for runtime address patching ─── */
static uint8_t g_page0_ram[CMT2310A_PAGE0_SIZE];

/* ─── Mutable page1 for runtime power + rate patching ─── */
static uint8_t g_page1_ram[CMT2310A_PAGE1_SIZE];

/* Patch page0 address-detection registers into RAM copy.
 * Page0 register map (idx = offset from 0x28):
 *   idx24 (0x40): addr detect mode / split mode / err mask config
 *   idx26 (0x42): dest addr value (RX filter — this node's address)
 *   idx28 (0x44): dest addr bit mask
 *   idx30 (0x46): src/loc addr value (TX source identity)
 *   idx32 (0x48): src addr bit mask
 * Derived from RFPDK_V1.66 cmt2310a_params-addressdetect.h diff.     */
static void patch_page0_addr() {
  memcpy(g_page0_ram, g_page0, CMT2310A_PAGE0_SIZE);
  if (g_addr_enabled) {
    /* 0x53 = Detect Address 0x00+0xFF | Split Dest+Loc | ErrMask on */
    g_page0_ram[24] = 0x53;
    g_page0_ram[26] = g_node_addr;   /* dest addr = our RX filter addr  */
    g_page0_ram[28] = 0xFF;          /* dest mask = exact match (all bits) */
    g_page0_ram[30] = g_node_addr;   /* src/loc addr = our TX identity  */
    g_page0_ram[32] = 0x00;          /* src mask = 0x00 = accept ANY source */
  }
  /* else: g_page0_ram stays as original (no addressing) */
}

#ifdef DEV_SWEEP
/* ── Deviation-sweep arrays ─────────────────────────────────────
 * All 4 slots = 1.2 kbps  865 MHz  Cap=31  AFC=on  [98]=0x18
 * Only deviation changes between slots.                        */

/* Slot 0 — Dev 0.6 kHz  (RFPDK cmt2310a_params-xtal-afc-1.2-600.h) */
static const uint8_t g_page1_d06[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0x69,0x00,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0x24,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,
    0x01,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Slot 1 — Dev 2.4 kHz  (RFPDK cmt2310a_params2.4k.h, [98] forced 0x18) */
static const uint8_t g_page1_d24[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0xA3,0x01,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0xB4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,
    0x05,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Slot 2 — Dev 3.6 kHz  (RFPDK cmt2310a_params-3.6khz.h, [98] forced 0x18) */
static const uint8_t g_page1_d36[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0x75,0x02,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0xB4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,
    0x07,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Slot 3 — Dev 5.0 kHz  (RFPDK cmt2310a_params-5khz.h, [98] forced 0x18) */
static const uint8_t g_page1_d50[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0x6A,0x03,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0xB4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,
    0x0A,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Lookup tables — deviation sweep */
static const uint8_t * const page1_table[4] = {
    g_page1_d06, g_page1_d24, g_page1_d36, g_page1_d50
};
static const char * const rate_labels[4] = { "D0.6", "D2.4", "D3.6", "D5.0" };
static const uint16_t      rate_bps[4]   = { 1200, 1200, 1200, 1200 };

#else /* normal multi-rate mode */

/* Page 1 — 1.200 kbps  865 MHz  Dev 2.4 kHz  Cap=31  [98]=0x18:RSSI@SYNC
 * Deviation regs from RFPDK_V1.66  cmt2310a_params2.4k.h  (sweep-validated)
 * idx27=0xA3 idx28=0x01 idx73=0xB4 idx84=0x05 */
static const uint8_t g_page1_1k2[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0xA3,0x01,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,  /* idx27=0xA3 idx28=0x01: dev 2.4kHz */
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0xB4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,  /* idx73=0xB4: filter BW */
    0x05,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,  /* idx84=0x05: dev scale */
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Page 1 — 1.200 kbps  865 MHz  Dev 3.6 kHz  Cap=31  [98]=0x18:RSSI@SYNC
 * From RFPDK cmt2310a_params-3.6khz.h — deviation comparison vs 2.4kHz
 * idx27=0x75 idx28=0x02 idx73=0xB4 idx84=0x07 */
static const uint8_t g_page1_1k2_36[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x75,0x02,0xE0,
    0x20,0x08,0x06,0x75,0x02,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x75,0x02,0x00,0x36,0x05,0x20,0xC8,0x63,0xA1,0x2B,0x68,0x58,
    0x40,0xB4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x00,0x39,0xE2,
    0x07,0x28,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Page 1 — 2.400 kbps  865 MHz  Dev 5.0 kHz  Cap=31  [98]=0x18:RSSI@SYNC
 * From RFPDK cmt2310a_params-2.4-5kdeviation.h  [98] forced 0x18
 * idx27=0x6A idx28=0x03 idx73=0x34 idx84=0x0A */
static const uint8_t g_page1_2k4[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0xEA,0x04,0xE0,
    0x20,0x08,0x05,0x6A,0x03,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0xEA,0x04,0x00,0x36,0x01,0x20,0xC8,0x63,0xA1,0x15,0x34,0x58,
    0x40,0x34,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x08,0x39,0xE2,
    0x0A,0x24,0x01,0xB5,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Page 1 — 9.600 kbps  865 MHz  Dev 10 kHz  Cap=31  [98]=0x18:RSSI@SYNC */
static const uint8_t g_page1_9k6[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0xA9,0x13,0xE0,
    0x20,0x08,0x04,0xD3,0x06,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0xA9,0x13,0x00,0x36,0x01,0x20,0xC8,0x63,0xA1,0x05,0x0D,0x58,
    0x40,0xC4,0x74,0xF0,0x0F,0x01,0x17,0xE6,0x54,0x0C,0x39,0xE2,
    0x14,0x0C,0x01,0xB4,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Page 1 — 19.200 kbps  865 MHz  Dev 10 kHz  Cap=31  [98]=0x18:RSSI@SYNC */
static const uint8_t g_page1_19k2[CMT2310A_PAGE1_SIZE] = {
    0x10,0x06,0x00,0xFF,0x00,0xCD,0x1F,0x20,0x50,0x87,0x31,0x5B,
    0x08,0x00,0xFF,0x00,0x6C,0x00,0x00,0x02,0xA4,0x52,0x27,0xE0,
    0x20,0x08,0x03,0xD3,0x06,0x00,0x91,0x7F,0x00,0x18,0x00,0x00,
    0x1F,0xE4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xC0,0x6C,0x00,0x40,0xD2,0x6B,0x00,0xC0,0x0F,0x64,0x06,0x02,
    0x52,0x27,0x00,0x36,0x01,0x20,0xC8,0x63,0xA1,0x83,0x06,0x58,
    0x40,0x44,0x74,0xE0,0x0F,0x01,0x17,0xE6,0x54,0x0C,0x39,0xE2,
    0x14,0x04,0x01,0xB4,0x0C,0x0F,0x00,0x4C,0x00,0x00,0xF6,0x00,
    0x00,0x00,0x18,0x81,0x00,0x00,0x47,0x12,0x25,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

/* Lookup tables — 5 rate/deviation combos */
#define NUM_RATES 5
static const uint8_t * const page1_table[NUM_RATES] = {
    g_page1_1k2, g_page1_1k2_36, g_page1_2k4, g_page1_9k6, g_page1_19k2
};
static const char * const rate_labels[NUM_RATES] = {
    "1.2/2.4", "1.2/3.6", "2.4/5", "9.6k", "19.2k"
};
static const uint16_t      rate_bps[NUM_RATES] = { 1200, 1200, 2400, 9600, 19200 };

#endif /* DEV_SWEEP */
#define NUM_SIZES 6
static const uint16_t      size_opts[NUM_SIZES] = { 16, 32, 64, 128, 250, 512 };
/* NOTE: 512B uses 2-byte length field + FIFO streaming (payload > 254B).
   Both TX and RX boards MUST select the same size for 512B tests.
   Sizes ≤ 250 use 1-byte length.  Merged FIFO (256B) with streaming for large. */

/* ═══════════════════════════════════════════════════════════════
   ███ BIT-BANG SPI                                           ███
   ═══════════════════════════════════════════════════════════════ */
/* ═══ Hardware SPI (ESP32-C3 SPI2/FSPI @ 4 MHz) ═══
 * CMT2310A supports up to 10 MHz.  8 MHz caused an intermittent bug: a
 * mid-test page1 reconfig (112-byte unverified burst write via bulk_wr)
 * silently dropped the data-rate bytes, freezing the radio at whatever
 * rate the FIRST config in that run used — later "9.6k"/"19.2k" configs
 * kept measuring 1.2kbps performance under the wrong label.  4 MHz is
 * still 2x the original 2 MHz and has not shown this failure.
 * Mode 0 (CPOL=0 CPHA=0): data set on falling edge, latched on rising. */
#define CMT_SPI_SPEED  4000000UL    /* 4 MHz — safe margin, still 2x original */
static SPISettings cmtSPI(CMT_SPI_SPEED, MSBFIRST, SPI_MODE0);

static void spi_init() {
  pinMode(CMT_CSB, OUTPUT);
  digitalWrite(CMT_CSB, HIGH);
  SPI.begin(CMT_SCLK, CMT_MISO, CMT_MOSI);   /* SCK=4, MISO=6, MOSI=5 */
}

static void spi_write(u8 addr, const u8 *buf, u16 len) {
  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);                    /* CS setup (official demo uses ~2-4 µs) */
  SPI.transfer(addr & 0x7F);
  for (u16 i = 0; i < len; i++) SPI.transfer(buf[i]);
  delayMicroseconds(1);                    /* CS hold */
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
}

static void spi_read(u8 addr, u8 *buf, u16 len) {
  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);
  SPI.transfer(addr | 0x80);
  for (u16 i = 0; i < len; i++) buf[i] = SPI.transfer(0xFF);
  delayMicroseconds(1);
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
}

/* ═══ Register helpers ═══ */
static u8   rr(u8 a)            { u8 v = 0xFF; spi_read(a, &v, 1); return v; }
static void rw(u8 a, u8 d)      { spi_write(a, &d, 1); }
static void rb(u8 a, u8 d, u8 m){ u8 v = rr(a); v &= ~m; v |= (d & m); spi_write(a, &v, 1); }
static void bulk_wr(const u8 *b, u16 l) { spi_write(CMT2310A_CRW_PORT, b, l); }

/* 2-byte length mode flag — set in startOperation based on data_len */
static bool g_2byte_len = false;

/* Write length field + first chunk of payload to FIFO.
 * Returns number of payload bytes actually written (may be < plen if FIFO full). */
static u16 fifo_wr_vlen(const u8 *payload, u16 plen) {
  u16 len_hdr  = g_2byte_len ? 2 : 1;
  u16 addr_hdr = g_addr_enabled ? 2 : 0;    /* dest + src address bytes */
  u16 max_data = FIFO_SIZE - len_hdr - addr_hdr;
  u16 chunk = (plen > max_data) ? max_data : plen;

  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMT2310A_FIFO_PORT & 0x7F);
  if (g_addr_enabled) {
    SPI.transfer(g_tx_dest_addr);            /* dest address byte */
    SPI.transfer(g_node_addr);               /* src/loc address byte */
  }
  if (g_2byte_len) {
    SPI.transfer((u8)(plen >> 8));           /* length high byte */
    SPI.transfer((u8)(plen & 0xFF));         /* length low byte  */
  } else {
    SPI.transfer((u8)plen);
  }
  for (u16 i = 0; i < chunk; i++) SPI.transfer(payload[i]);
  delayMicroseconds(1);
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
  if (g_addr_debug && g_addr_enabled) {
    Serial.printf("[FIFO-TX] dest=0x%02X src=0x%02X len=%u first4=[%02X %02X %02X %02X]\n",
                  g_tx_dest_addr, g_node_addr, plen,
                  chunk > 0 ? payload[0] : 0, chunk > 1 ? payload[1] : 0,
                  chunk > 2 ? payload[2] : 0, chunk > 3 ? payload[3] : 0);
  }
  return chunk;
}

/* Read length field + payload from FIFO (for packets that fit entirely). */
static u16 fifo_rd_vlen(u8 *payload, u16 maxlen) {
  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMT2310A_FIFO_PORT | 0x80);
  SPI.transfer(0xFF);            /* dummy byte */
  if (g_addr_enabled) {
    g_rx_dest_addr = SPI.transfer(0xFF);  /* dest addr (ours or broadcast) */
    g_rx_src_addr  = SPI.transfer(0xFF);  /* src addr (who sent it)        */
  }
  u16 len;
  if (g_2byte_len) {
    u8 lh = SPI.transfer(0xFF);
    u8 ll = SPI.transfer(0xFF);
    len = ((u16)lh << 8) | ll;
  } else {
    len = SPI.transfer(0xFF);
  }
  if (len > maxlen) len = maxlen;
  for (u16 i = 0; i < len; i++) payload[i] = SPI.transfer(0xFF);
  delayMicroseconds(1);
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
  if (g_addr_debug && g_addr_enabled) {
    Serial.printf("[FIFO-RX] dest=0x%02X src=0x%02X len=%u first4=[%02X %02X %02X %02X]\n",
                  g_rx_dest_addr, g_rx_src_addr, len,
                  len > 0 ? payload[0] : 0, len > 1 ? payload[1] : 0,
                  len > 2 ? payload[2] : 0, len > 3 ? payload[3] : 0);
  }
  return len;
}

/* Write raw payload bytes to TX FIFO (no length field — for streaming continuation). */
static void fifo_wr_raw(const u8 *data, u16 count) {
  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMT2310A_FIFO_PORT & 0x7F);
  for (u16 i = 0; i < count; i++) SPI.transfer(data[i]);
  delayMicroseconds(1);
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
}

/* Read raw bytes from RX FIFO (no length field parsing — for streaming drain). */
static u16 fifo_rd_raw(u8 *buf, u16 count) {
  SPI.beginTransaction(cmtSPI);
  digitalWrite(CMT_CSB, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMT2310A_FIFO_PORT | 0x80);
  SPI.transfer(0xFF);            /* dummy byte */
  for (u16 i = 0; i < count; i++) buf[i] = SPI.transfer(0xFF);
  delayMicroseconds(1);
  digitalWrite(CMT_CSB, HIGH);
  SPI.endTransaction();
  return count;
}

/* ═══════════════════════════════════════════════════════════════
   ███ CMT2310A DRIVER                                        ███
   ═══════════════════════════════════════════════════════════════ */
static void sel_page(u8 p) { rb(CMT2310A_CTL_REG_126, (u8)(p << CMT_BIT6), M_HV_PAGE_SEL); }
static u8 chip_state()     { return rr(CMT2310A_CTL_REG_10); }
static void load_page(u8 p, const u8 *d, u8 l) { sel_page(p); bulk_wr(d, l); sel_page(0); }

/* FIFO direction — original (proven working) sense: TX=clear, RX=set.
 * This is inverted from the official demo naming but matches our chip behaviour. */
static void fifo_dir_tx() { rb(CMT2310A_CTL_REG_19, 0,                M_FIFO_TX_RX_SEL); }
static void fifo_dir_rx() { rb(CMT2310A_CTL_REG_19, M_FIFO_TX_RX_SEL, M_FIFO_TX_RX_SEL); }

static BOOL wait_state(u8 cmd, u8 want) {
  rw(CMT2310A_CTL_REG_01, cmd); u32 t = millis();
  while (millis() - t < 50) { delay(1); feedLoop(); if (chip_state() == want) return TRUE; }
  return FALSE;
}
static BOOL go_ready() { return wait_state(M_GO_READY, STATE_IS_READY); }
static BOOL go_tx()    { return wait_state(M_GO_TX,    STATE_IS_TX);    }
static BOOL go_rx()    { return wait_state(M_GO_RX,    STATE_IS_RX);    }

static BOOL api_wait(u8 cmd) {
  rw(CMT2310A_CTL_REG_08, cmd);
  u8 exp = (u8)(M_API_CMD_FLAG | cmd); u32 t = millis();
  while (millis() - t < 2000) { delay(2); feedLoop(); if (rr(CMT2310A_CTL_REG_09) == exp) return TRUE; }
  return FALSE;
}
static void xo_cfg(u8 div) {
  u8 v = (u8)((div & 3) << 6); sel_page(0);
  rb(CMT2310A_CTL_REG_07, v, CMT2310A_CTL_REG_07_MASK); sel_page(0);
}
/* Set payload length register (REG_61 = low byte, REG_62 = high byte).
 * In 1-byte length mode REG_62 is ignored by the chip but we write it anyway. */
static void set_plen(u16 len) {
  rw(CMT2310A_CTL_REG_61, (u8)(len & 0xFF));
  rw(CMT2310A_CTL_REG_62, (u8)(len >> 8));
}

static BOOL clr_tx_flag() {
  u32 t = millis(); rw(CMT2310A_CTL_REG_24, M_TX_DONE_CLR);
  while (rr(CMT2310A_CTL_REG_24) & M_TX_DONE_FLG) {
    delay(1); feedLoop();
    if (millis() - t > 20) { Serial.println("[!] TX flag stuck"); return FALSE; }
  }
  return TRUE;
}
static BOOL clr_pkt_flag() {
  u32 t = millis(); rw(CMT2310A_CTL_REG_25, M_PKT_DONE_CLR);
  while (rr(CMT2310A_CTL_REG_26) & M_PKT_DONE_FLG) {
    delay(1); feedLoop();
    if (millis() - t > 20) { Serial.println("[!] PKT flag stuck"); return FALSE; }
  }
  return TRUE;
}

/* ─── RSSI ─── */
static uint8_t g_rssi_raw = 0;
static int16_t readRSSI() {
  g_rssi_raw = rr(CMT2310A_CTL_REG_34);
  int8_t dbm = (int8_t)g_rssi_raw;
  return (int16_t)dbm;
}

/* ─── Chip temperature sensor ─── */
static uint8_t readChipTemp() {
  return rr(CMT2310A_CTL_REG_36);
}

static void radio_init(uint8_t rateIdx);

static void rf_config(uint8_t rateIdx) {
  patch_page0_addr();
  load_page(0, g_page0_ram, CMT2310A_PAGE0_SIZE);

  /* Build page1: rate registers + TX power patch */
  memcpy(g_page1_ram, page1_table[rateIdx], CMT2310A_PAGE1_SIZE);
  /* Patch PA power registers (page1 offsets 30,31,33,36 = regs 0x9E,0x9F,0xA1,0xA4) */
  g_page1_ram[30] = pwr_regs[cfg_pwrIdx][0];  /* 0x9E */
  g_page1_ram[31] = pwr_regs[cfg_pwrIdx][1];  /* 0x9F */
  g_page1_ram[33] = pwr_regs[cfg_pwrIdx][2];  /* 0xA1 */
  g_page1_ram[36] = pwr_regs[cfg_pwrIdx][3];  /* 0xA4 */
  load_page(1, g_page1_ram, CMT2310A_PAGE1_SIZE);
  xo_cfg(2);

  rw(CMT2310A_CTL_REG_00, 0x03); delay(5); feedLoop();
  go_ready(); delay(2);
  api_wait(0x02); delay(2); api_wait(0x01);

  /* Variable-length mode + LENGTH_SIZE selection */
  if (g_2byte_len)
    rb(CMT2310A_CTL_REG_63, (u8)(M_PKT_TYPE | M_LENGTH_SIZE),
                              (u8)(M_PKT_TYPE | M_LENGTH_SIZE));
  else
    rb(CMT2310A_CTL_REG_63, M_PKT_TYPE, (u8)(M_PKT_TYPE | M_LENGTH_SIZE));

  { u16 maxpl = g_2byte_len ? 514 : 255;
    if (g_addr_enabled) maxpl += 2;   /* radio counts addr bytes in plen */
    set_plen(maxpl);
  }

  rb(CMT2310A_CTL_REG_19,
     (u8)(M_FIFO_MERGE_EN | M_FIFO_AUTO_CLR_RX),
     (u8)(M_FIFO_MERGE_EN | M_FIFO_AUTO_CLR_RX));  /* merged FIFO (256B), auto-clear RX */
  rw(CMT2310A_CTL_REG_20, FIFO_TH_VAL);  /* FIFO threshold for streaming */
  fifo_dir_tx();

  rb(CMT2310A_CTL_REG_04, GPIO0_SEL_INT1, M_GPIO0_SEL);
  rb(CMT2310A_CTL_REG_04, (u8)(GPIO1_SEL_INT2 << 3), M_GPIO1_SEL);
  rb(CMT2310A_CTL_REG_16, INT_SRC_TX_DONE,  M_INT1_SEL);
  rb(CMT2310A_CTL_REG_17, INT_SRC_PKT_DONE, M_INT2_SEL);
  rb(CMT2310A_CTL_REG_17, 0, M_INT1_POLAR | M_INT2_POLAR);

  rw(CMT2310A_CTL_REG_18,
     M_TX_DONE_EN | M_PREAM_PASS_EN | M_SYNC_PASS_EN |
     M_ADDR_PASS_EN | M_CRC_PASS_EN | M_PKT_DONE_EN);
  rw(CMT2310A_CTL_REG_14,
     M_RX_FIFO_FULL_RX_EN | M_RX_FIFO_NMTY_RX_EN |
     M_RX_FIFO_TH_RX_EN  | M_RX_FIFO_OVF_EN     |
     M_TX_FIFO_FULL_EN   | M_TX_FIFO_NMTY_EN    |
     M_TX_FIFO_TH_EN);

  rb(CMT2310A_CTL_REG_22, 0, M_PA_DIFF_SEL);  /* single-ended PA (mode 0, per official demo) */
  Serial.printf("[rf_config] rate=%s  pwr=%sdBm  state=0x%02X  FIFO=merged(%uB)  TH=%u  len=%s",
                rate_labels[rateIdx], pwr_labels[cfg_pwrIdx],
                chip_state(), FIFO_SIZE, FIFO_TH_VAL,
                g_2byte_len ? "2-byte" : "1-byte");
  if (g_addr_enabled) {
    Serial.printf("  ADDR=0x%02X  dest=0x%02X", g_node_addr, g_tx_dest_addr);
    Serial.printf("\n[rf_config] page0 addr regs: idx24=0x%02X idx26=0x%02X idx28=0x%02X idx30=0x%02X idx32=0x%02X",
                  g_page0_ram[24], g_page0_ram[26], g_page0_ram[28], g_page0_ram[30], g_page0_ram[32]);
  } else {
    Serial.print("  ADDR=off");
  }
  Serial.println();
}

/* Chip existence check — write 0xAA to a scratch register, read back (per official demo) */
static BOOL cmt2310a_is_exist() {
  u8 back = rr(0x0C);            /* CTL_REG_12 — safe scratch register */
  rw(0x0C, 0xAA);
  u8 readback = rr(0x0C);
  rw(0x0C, back);                /* restore original */
  return (readback == 0xAA);
}

static void radio_init(uint8_t rateIdx) {
  spi_init();

  /* Verify chip is alive BEFORE soft reset (per official demo order) */
  if (!cmt2310a_is_exist()) {
    Serial.println("[radio_init] CMT2310A NOT FOUND!");
    oledStatus("RF ERROR", "CMT2310A not found!", "", "Check SPI wiring");
    ledRed();
    while (true) { feedLoop(); delay(100); }   /* halt */
  }

  rw(CMT2310A_SOFT_RST, 0xFF); delay(20); feedLoop();
  u8 p = rr(CMT2310A_SOFT_RST);
  Serial.printf("[radio_init] probe=0x%02X chip=%s\n", p, p == 0xFF ? "ERROR" : "OK");
  rf_config(rateIdx);
}

/* ═══════════════════════════════════════════════════════════════
   ███ TRX STATE MACHINE                                      ███
   ═══════════════════════════════════════════════════════════════ */
#define MAX_PAYLOAD 520            /* 2 (PKT_HDR) + 512 (max data) + headroom */

static TRXState g_st    = ST_IDLE;
static u8  *g_tbuf      = NULL;  static u16 g_tlen  = 0;
static u32  g_ttout     = INFINITE; static u32 g_ttick = 0;
static u8  *g_rbuf      = NULL;  static u16 g_rmax  = 0;
static u32  g_rtout     = INFINITE; static u32 g_rtick = 0;
u16     g_rxd_len       = 0;

/* FIFO streaming state */
static u16  g_stream_off = 0;        /* TX: payload bytes written so far */
static u16  g_rx_off     = 0;        /* RX: payload bytes drained so far */
static u16  g_rx_total   = 0;        /* RX: total payload bytes (from length field) */
static bool g_rx_streaming = false;  /* RX: large packet, needs mid-reception drain */

static void rf_start_tx(u8 *buf, u16 len, u32 tms) {
  if (g_st != ST_IDLE) return;
  g_tbuf = buf; g_tlen = len; g_ttout = tms; g_st = ST_TX_START;
}
static void rf_start_rx(u8 *buf, u16 mx, u32 tms) {
  if (g_st != ST_IDLE) return;
  g_rbuf = buf; g_rmax = mx; g_rtout = tms; g_st = ST_RX_START;
}

static RFResult rf_process() {
  RFResult res = RF_BUSY;
  switch (g_st) {
    case ST_IDLE: return RF_IDLE;

    case ST_TX_START:{
      go_ready(); delay(2);
      fifo_dir_tx();
      if (!clr_tx_flag()) { g_st = ST_ERROR; break; }
      rb(CMT2310A_CTL_REG_27, M_TX_FIFO_CLR, M_TX_FIFO_CLR);
      /* set_plen must include addr header — radio counts addr bytes against plen */
      set_plen(g_tlen + (g_addr_enabled ? 2 : 0));

      /* Write length field + first chunk to FIFO */
      g_stream_off = fifo_wr_vlen(g_tbuf, g_tlen);

      if (!(rr(CMT2310A_CTL_REG_28) & M_TX_FIFO_NMTY_FLG))
        { Serial.println("[TX] FIFO empty"); g_st = ST_ERROR; break; }
      if (!go_tx())
        { Serial.printf("[TX] failed 0x%02X\n", chip_state()); g_st = ST_ERROR; break; }
      g_ttick = millis();
      /* Need streaming? */
      g_st = (g_stream_off < g_tlen) ? ST_TX_STREAM : ST_TX_WAIT;
      if (g_st == ST_TX_STREAM)
        Serial.printf("[TX-DBG] streaming: first=%u/%u  FIFO=0x%02X\n",g_stream_off, g_tlen, rr(CMT2310A_CTL_REG_28));
      break;
    }

    case ST_TX_STREAM: {
      /* Refill FIFO when it drains below threshold.
       * TX_FIFO_TH_FLG == 0 means unsent data < FIFO_TH_VAL → room to write. */
      u8 f28 = rr(CMT2310A_CTL_REG_28);
      if (!(f28 & M_TX_FIFO_TH_FLG)) {
        u16 remain = g_tlen - g_stream_off;
        u16 room   = FIFO_SIZE - FIFO_TH_VAL;   /* conservative free space */
        u16 chunk  = (remain > room) ? room : remain;
        fifo_wr_raw(g_tbuf + g_stream_off, chunk);
        g_stream_off += chunk;
        Serial.printf("[TX-DBG] refill +%u → %u/%u\n", chunk, g_stream_off, g_tlen);
        if (g_stream_off >= g_tlen) {
          g_st = ST_TX_WAIT;                     /* all data in FIFO, wait TX_DONE */
          break;
        }
      }
      /* Check for TX_DONE during stream (underflow error) */
      if (digitalRead(CMT_GPIO0) || (rr(CMT2310A_CTL_REG_24) & M_TX_DONE_FLG) ||
          chip_state() != STATE_IS_TX) {
        Serial.println("[TX] FIFO underflow — TX_DONE during stream");
        g_st = ST_TX_DONE_ST; break;
      }
      /* Timeout */
      if (g_ttout != INFINITE && millis() - g_ttick > g_ttout)
        { Serial.println("[TX] stream timeout"); g_st = ST_TX_TIMEOUT; }
      break;
    }

    case ST_TX_WAIT: {
      /* Check 3 ways: GPIO pin, flag register, OR chip left TX state */
      u8 cs = chip_state();
      if (digitalRead(CMT_GPIO0) ||
          (rr(CMT2310A_CTL_REG_24) & M_TX_DONE_FLG) ||
          (cs != STATE_IS_TX && millis() - g_ttick > 5))  /* chip no longer in TX = done */
        { g_st = ST_TX_DONE_ST; break; }
      if (g_ttout != INFINITE && millis() - g_ttick > g_ttout)
        { Serial.printf("[TX] timeout cs=0x%02X flg=0x%02X gpio=%d\n",
                        cs, rr(CMT2310A_CTL_REG_24), digitalRead(CMT_GPIO0));
          g_st = ST_TX_TIMEOUT; }
      break;
    }

    case ST_TX_DONE_ST:
      clr_tx_flag();
      go_ready(); delay(2); g_st = ST_IDLE; res = RF_TX_DONE; break;

    case ST_TX_TIMEOUT:
      go_ready(); delay(2); g_st = ST_IDLE; res = RF_TX_TIMEOUT; break;

    case ST_RX_START: {
      go_ready(); delay(2);
      fifo_dir_rx();
      /* Clear all RX event flags — matches official CMOSTEK demo.
       * Write all 5 clear bits first, then use polling clr_pkt_flag()
       * to verify PKT_DONE actually cleared (needs 1-2 SPI cycles). */
      rw(CMT2310A_CTL_REG_25,
         (u8)(M_PREAM_PASS_CLR | M_SYNC_PASS_CLR | M_ADDR_PASS_CLR |
              M_CRC_PASS_CLR   | M_PKT_DONE_CLR));
      if (!clr_pkt_flag()) { g_st = ST_ERROR; break; }
      rb(CMT2310A_CTL_REG_27, M_RX_FIFO_CLR, M_RX_FIFO_CLR);
      { u16 maxpl = g_2byte_len ? 514 : 255;
        if (g_addr_enabled) maxpl += 2;   /* radio counts addr bytes in plen */
        set_plen(maxpl);
      }

      /* Enable RX streaming only when packets can exceed FIFO.
       * With merged 256B FIFO and TH=200, packets ≤199 bytes never trigger
       * streaming and use the clean fifo_rd_vlen() path. */
      g_rx_off = 0;
      g_rx_total = 0;
      g_rx_streaming = g_2byte_len;   /* only >250-byte payloads need streaming */

      if (!go_rx())
        { Serial.printf("[RX] failed 0x%02X\n", chip_state()); g_st = ST_ERROR; break; }
      g_rtick = millis(); g_st = ST_RX_WAIT;
      break;
    }

    case ST_RX_WAIT: {
      /* For large packets, drain FIFO mid-reception to prevent overflow */
      if (g_rx_streaming) {
        u8 f28 = rr(CMT2310A_CTL_REG_28);
        if (f28 & M_RX_FIFO_TH_FLG) {           /* unread data > TH */
          if (g_rx_off == 0) {
            /* First drain — includes addr bytes (if enabled) + length field */
            u16 addr_hdr = g_addr_enabled ? 2 : 0;
            u16 len_hdr  = g_2byte_len ? 2 : 1;
            u16 data_bytes = FIFO_TH_VAL - addr_hdr - len_hdr;
            SPI.beginTransaction(cmtSPI);
            digitalWrite(CMT_CSB, LOW);
            delayMicroseconds(1);
            SPI.transfer(CMT2310A_FIFO_PORT | 0x80);
            SPI.transfer(0xFF);            /* dummy byte */
            if (g_addr_enabled) {
              g_rx_dest_addr = SPI.transfer(0xFF);
              g_rx_src_addr  = SPI.transfer(0xFF);
            }
            if (g_2byte_len) {
              u8 lh = SPI.transfer(0xFF);
              u8 ll = SPI.transfer(0xFF);
              g_rx_total = ((u16)lh << 8) | ll;
            } else {
              g_rx_total = SPI.transfer(0xFF);
            }
            if (data_bytes > g_rmax) data_bytes = g_rmax;
            for (u16 i = 0; i < data_bytes; i++) g_rbuf[i] = SPI.transfer(0xFF);
            delayMicroseconds(1);
            digitalWrite(CMT_CSB, HIGH);
            SPI.endTransaction();
            g_rx_off = data_bytes;
            Serial.printf("[RX-DBG] 1st drain: len=%u  got=%u  from=0x%02X\n",
                          g_rx_total, g_rx_off, g_rx_src_addr);
          } else {
            /* Subsequent drain — all payload data */
            u16 chunk = FIFO_TH_VAL;
            u16 remain = (g_rx_total > g_rx_off) ? (g_rx_total - g_rx_off) : 0;
            if (chunk > remain) chunk = remain;
            if (chunk > 0) {
              fifo_rd_raw(&g_rbuf[g_rx_off], chunk);
              g_rx_off += chunk;
              Serial.printf("[RX-DBG] drain +%u → %u/%u\n", chunk, g_rx_off, g_rx_total);
            }
          }
        }
      }

      if (digitalRead(CMT_GPIO1) || (rr(CMT2310A_CTL_REG_26) & M_PKT_DONE_FLG))
        { g_st = ST_RX_DONE_ST; break; }
      if (g_rtout != INFINITE && millis() - g_rtick > g_rtout)
        { go_ready(); delay(2); g_st = ST_IDLE; res = RF_RX_TIMEOUT; }
      break;
    }

    case ST_RX_DONE_ST: {
      g_last_rssi = readRSSI();
      clr_pkt_flag();

      if (g_rx_streaming && g_rx_off > 0) {
        /* Read remaining bytes from FIFO after PKT_DONE */
        u16 remain = (g_rx_total > g_rx_off) ? (g_rx_total - g_rx_off) : 0;
        if (remain > (g_rmax - g_rx_off)) remain = g_rmax - g_rx_off;
        if (remain > 0) {
          fifo_rd_raw(&g_rbuf[g_rx_off], remain);
          g_rx_off += remain;
        }
        g_rxd_len = g_rx_off;
        Serial.printf("[RX-DBG] DONE: total=%u  got=%u  rxd_len=%u\n",
                      g_rx_total, g_rx_off, g_rxd_len);
      } else {
        /* Small packet — read entire FIFO at once (original path) */
        g_rxd_len = fifo_rd_vlen(g_rbuf, g_rmax);
      }

      /* Debug: dump raw received bytes for FIFO format verification */
      if (g_addr_debug) {
        Serial.printf("[RX-DUMP] rxd_len=%u  RSSI=%d  hex:", g_rxd_len, (int)g_last_rssi);
        for (u16 i = 0; i < g_rxd_len && i < 16; i++)
          Serial.printf(" %02X", g_rbuf[i]);
        if (g_rxd_len > 16) Serial.print(" ...");
        Serial.println();
      }

      /* Auto-swap: reply goes to whoever sent this packet
         (skip if mesh relay — routing table handles dest) */
      if (g_addr_enabled && !g_mesh_on) g_tx_dest_addr = g_rx_src_addr;

      go_ready(); delay(2);
      g_st = ST_IDLE; res = RF_RX_DONE;
      break;
    }

    case ST_ERROR:
      Serial.println("[RF] error — re-init");
      g_st = ST_IDLE; res = RF_ERROR; break;
  }
  return res;
}

/* ═══════════════════════════════════════════════════════════════
   ███ MENU SYSTEM (scrollable)                               ███
   ═══════════════════════════════════════════════════════════════ */

#define MENU_ITEMS 9
/* Item indices */
#define MI_MODE   0
#define MI_RATE   1
#define MI_PWR    2
#define MI_SIZE   3
#define MI_START  4
#define MI_PER    5
#define MI_CW     6
#define MI_REGDUMP 7
#define MI_AUTO_RANGE 8

static void drawMenu(uint8_t cursor, uint8_t scroll) {
  if (!_oledOK) return;
  char buf[22];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 12, "RF Test Setup");
  u8g2.setFont(u8g2_font_6x10_tr);

  for (uint8_t i = 0; i < 4 && (scroll + i) < MENU_ITEMS; i++) {
    uint8_t item = scroll + i;
    char arrow = (item == cursor) ? '>' : ' ';
    int y = 24 + i * 11;
    switch (item) {
      case MI_MODE:
        snprintf(buf, sizeof(buf), "%cMode: %s", arrow,
                 cfg_mode <= MODE_TRX ? mode_labels[cfg_mode] : mode_labels[0]);
        break;
      case MI_RATE:
        snprintf(buf, sizeof(buf), "%cRate: %s kbps", arrow, rate_labels[cfg_rateIdx]);
        break;
      case MI_PWR:
        snprintf(buf, sizeof(buf), "%cPower: %s dBm", arrow, pwr_labels[cfg_pwrIdx]);
        break;
      case MI_SIZE:
        snprintf(buf, sizeof(buf), "%cSize: %u bytes", arrow, size_opts[cfg_sizeIdx]);
        break;
      case MI_START:
        snprintf(buf, sizeof(buf), "%c[START TEST]", arrow);
        break;
      case MI_PER:
        snprintf(buf, sizeof(buf), "%cPER 1000 Test", arrow);
        break;
      case MI_CW:
        snprintf(buf, sizeof(buf), "%cCW Carrier TX", arrow);
        break;
      case MI_REGDUMP:
        snprintf(buf, sizeof(buf), "%cRegister Dump", arrow);
        break;
      case MI_AUTO_RANGE:
        snprintf(buf, sizeof(buf), "%cAuto Range Test", arrow);
        break;
    }
    u8g2.drawStr(0, y, buf);
  }

  /* Scroll indicator */
  if (MENU_ITEMS > 4) {
    int bar_h = 44 * 4 / MENU_ITEMS;
    int bar_y = 20 + (44 * scroll) / MENU_ITEMS;
    u8g2.drawFrame(125, 20, 3, 44);
    u8g2.drawBox(125, bar_y, 3, bar_h);
  }
  u8g2.sendBuffer();
}

static void runMenu() {
  uint8_t cursor = 0, scroll = 0;
  drawMenu(cursor, scroll);
  Serial.println("\n=== RF Test Setup — MENU ===");

  while (true) {
    feedLoop();
    Buttons b = readButtons();
    bool changed = false;

    if (b.up) {
      if (cursor > 0) cursor--;
      if (cursor < scroll) scroll = cursor;
      changed = true;
    }
    if (b.down) {
      if (cursor < MENU_ITEMS - 1) cursor++;
      if (cursor >= scroll + 4) scroll = cursor - 3;
      changed = true;
    }
    if (b.left || b.right) {
      int dir = b.right ? 1 : -1;
      switch (cursor) {
        case MI_MODE: cfg_mode = (cfg_mode + 3 + dir) % 3; break;  /* TX/RX/TRX */
        case MI_RATE: cfg_rateIdx = (cfg_rateIdx + NUM_RATES + dir) % NUM_RATES; break;
        case MI_PWR:  cfg_pwrIdx  = (cfg_pwrIdx + NUM_PWR_LEVELS + dir) % NUM_PWR_LEVELS; break;
        case MI_SIZE: cfg_sizeIdx = (cfg_sizeIdx + NUM_SIZES + dir) % NUM_SIZES; break;
      }
      changed = true;
    }
    if (b.ok) {
      ledFlashGreen(60);
      if (cursor == MI_START) {
        Serial.printf("[menu] Start: Mode=%s Rate=%s Pwr=%sdBm Size=%uB\n",
                      mode_labels[cfg_mode], rate_labels[cfg_rateIdx],
                      pwr_labels[cfg_pwrIdx], size_opts[cfg_sizeIdx]);
        return;  /* start operation */
      }
      else if (cursor == MI_PER) {
        cfg_mode = MODE_PER;
        Serial.printf("[menu] PER Test: Rate=%s Pwr=%sdBm Size=%uB\n",
                      rate_labels[cfg_rateIdx], pwr_labels[cfg_pwrIdx], size_opts[cfg_sizeIdx]);
        return;
      }
      else if (cursor == MI_CW) {
        cfg_mode = MODE_CW;
        Serial.println("[menu] CW Carrier TX");
        return;
      }
      else if (cursor == MI_REGDUMP) {
        cfg_mode = MODE_REG_DUMP;
        return;
      }
      else if (cursor == MI_AUTO_RANGE) {
        cfg_mode = MODE_AUTO_RANGE;
        Serial.println("[menu] Auto Range Test");
        return;
      }
    }

    if (changed) drawMenu(cursor, scroll);
    delay(5);
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ APPLICATION — COMMON                                   ███
   ═══════════════════════════════════════════════════════════════ */
static u8  pkt[MAX_PAYLOAD];       /* TX packet build buffer */
static u8  rxbuf[MAX_PAYLOAD];     /* RX receive buffer */
static u8  txData[MAX_PAYLOAD];    /* ACK / echo-back buffer */

static uint16_t data_len  = 64;
static char     dest_id   = 'B';
static char     my_id     = 'A';

#define MAX_RETRY 3
static u32 ack_timeout_ms = 2000;
static u32 tx_timeout_ms  = 500;

/*  ACK timeout = round-trip: TX air + RX processing + ACK air + margin
 *
 *  On-air packet bytes:
 *    Preamble  32 B   (RFPDK: Tx Size=32, Unit=8-bit)
 *    Sync       3 B   (RFPDK: 3-byte S2LP)
 *    Length     1 or 2 B  (variable-length mode)
 *    Payload    PKT_HDR+data_len  (dest + seq + data)
 *    ────────────────────────
 *    Total    = 35 + len_bytes + PKT_HDR + data_len
 *             = 37 + data_len  (1-byte len, ≤250B)
 *             = 38 + data_len  (2-byte len, 512B)
 *
 *  ACK is same size (RX echoes seq + payload back).
 */
static u32 calcACKTimeout() {
  u32 len_bytes = g_2byte_len ? 2 : 1;
  u32 extra     = g_mesh_on ? MESH_HDR : 0;
  u32 pkt_bytes = 35 + len_bytes + extra + PKT_HDR + data_len;
  u32 air_ms    = (pkt_bytes * 8UL * 1000UL) / rate_bps[cfg_rateIdx];
  u32 tmo       = 2 * air_ms + 100;         /* round-trip + processing */
  if (g_mesh_on) {
    /* Scale for multi-hop: each hop adds air_time + relay processing */
    tmo = (2 * MESH_MAX_TTL) * (air_ms + MESH_RELAY_DELAY) + 300;
  }
  if (tmo < 200) tmo = 200;
  return tmo;
}

/* TX timeout — single packet air time + margin (for TX_DONE GPIO0 check) */
static u32 calcTXTimeout() {
  u32 len_bytes = g_2byte_len ? 2 : 1;
  u32 pkt_bytes = 35 + len_bytes + PKT_HDR + data_len;
  u32 air_ms    = (pkt_bytes * 8UL * 1000UL) / rate_bps[cfg_rateIdx];
  u32 tmo       = air_ms + 100;
  if (tmo < 150) tmo = 150;
  return tmo;
}

/* ═══════════════════════════════════════════════════════════════
   ███ TX MODE                                                ███
   ═══════════════════════════════════════════════════════════════ */

static void updateOLED_TX() {
  if (millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  if (!_oledOK) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];

  /* Line 1: mode + config */
  snprintf(buf, sizeof(buf), "TX %s %uB %sdB", rate_labels[cfg_rateIdx], data_len, pwr_labels[cfg_pwrIdx]);
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 12, buf);

  u8g2.setFont(u8g2_font_6x10_tr);
  /* Line 2: Ok + failure breakdown  T=timeout(no ACK) C=corrupt(bad data) */
  snprintf(buf, sizeof(buf), "Ok:%lu T:%lu C:%lu",
           (unsigned long)ok_count,
           (unsigned long)fail_count,
           (unsigned long)corrupt_count);
  u8g2.drawStr(0, 24, buf);

  /* Line 3: RSSI + margin */
  int16_t margin = g_last_rssi - NOISE_FLOOR_DBM;
  snprintf(buf, sizeof(buf), "RSSI:%d M:%ddB %s",
           (int)g_last_rssi, (int)margin, rssiQ(g_last_rssi));
  u8g2.drawStr(0, 35, buf);

  /* Line 4: loss% + RTT + packet# */
  snprintf(buf, sizeof(buf), "L:%.0f%% R:%lu #%lu",
           lossRingPercent(), (unsigned long)g_last_rtt,
           (unsigned long)round_num);
  u8g2.drawStr(0, 46, buf);

  /* RSSI bar graph — bottom */
  drawRSSIBar(g_last_rssi);

  u8g2.sendBuffer();
}

static void send_test_packet() {
  round_num++;

  u16 moff = 0;                          /* mesh header offset into pkt[] */
  if (g_mesh_on) {
    mesh_set_radio_dest(g_mesh_final_dest);  /* route table → radio dest */
    pkt[0] = g_mesh_final_dest;              /* mesh final destination   */
    pkt[1] = g_node_addr;                    /* mesh origin = me         */
    pkt[2] = 0;                              /* hop_count = 0            */
    pkt[3] = MESH_MAX_TTL;                   /* TTL                      */
    pkt[4] = g_mesh_tx_seq++;                /* dedup seq                */
    mesh_mark_seen(g_node_addr, pkt[4]);     /* mark own pkt seen        */
    moff = MESH_HDR;
  }

  pkt[moff + 0] = (u8)dest_id;
  pkt[moff + 1] = tx_seq;
  for (uint16_t i = 0; i < data_len; i++) pkt[moff + PKT_HDR + i] = (u8)(i & 0xFF);

  u16 total = moff + PKT_HDR + data_len;

  Serial.printf("\n[TX] #%lu → %c seq=%u %uB",
                (unsigned long)round_num, dest_id, tx_seq, data_len);
  if (g_addr_enabled)
    Serial.printf("  radio→0x%02X", g_tx_dest_addr);
  if (g_mesh_on)
    Serial.printf("  mesh→0x%02X", g_mesh_final_dest);
  Serial.println();
  ledFlashGreen(40);
  rf_start_tx(pkt, total, tx_timeout_ms);
}

static void tx_loop() {
  feedLoop();
  /* Stop button */
  { Buttons b = readButtons();
    if (b.ok) {
      go_ready(); delay(2);
      Serial.println("[TX] STOPPED by user");
      drawStoppedScreen();
      appState = APP_STOPPED; delay(100); return;
    }
  }

  RFResult res = rf_process();

  if (res == RF_TX_DONE) {
    sent_at = millis();
    Serial.println("[TX] Sent — listening for ACK...");
    rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
  }
  else if (res == RF_RX_DONE) {
    u32 rtt = millis() - sent_at;
    g_last_rtt = rtt;

    u16 moff = 0;
    if (g_mesh_on && g_rxd_len >= MESH_HDR) {
      uint8_t am_dest = rxbuf[0], am_src = rxbuf[1];
      uint8_t am_hops = rxbuf[2], am_ttl = rxbuf[3];
      Serial.printf("jaa rha hu path update krne: dest. 0x%02X , via:  0x%02X\n",am_dest,g_rx_src_addr);
      /* Validate ACK mesh header — reject corrupted ACKs */
      if (!mesh_hdr_valid(am_dest, am_src, am_hops, am_ttl)) {
        Serial.printf("[MESH] BAD ACK HEADER: dest=0x%02X src=0x%02X hops=%u ttl=%u — ignoring mesh\n",
                      am_dest, am_src, am_hops, am_ttl);
        /* Still process ACK payload (skip mesh header) but don't learn garbage routes */
        moff = MESH_HDR;
      } else {
        /* Un-blacklist radio_src — it just proved it's alive */
        for (int bi = 0; bi < MESH_BLACKLIST_MAX; bi++) {
          if (g_blacklist[bi].active && g_blacklist[bi].addr == g_rx_src_addr) {
            g_blacklist[bi].active = false;
            Serial.printf("[MESH] Un-blacklisted 0x%02X (alive — sent ACK)\n", g_rx_src_addr);
          }
        }
        mesh_route_update(am_dest, g_rx_src_addr, am_hops + 1, g_last_rssi);
        if (g_rx_src_addr != am_src)
          mesh_route_update(g_rx_src_addr, g_rx_src_addr, 1, g_last_rssi);
        moff = MESH_HDR;
        g_mesh_fail_cnt = 0;  /* ACK received — reset route failure counter */
      }
    }

    char src = (char)rxbuf[moff + 0];
    u8 ack_seq = rxbuf[moff + 1];
    u16 dl = (g_rxd_len > moff + PKT_HDR) ? (g_rxd_len - moff - PKT_HDR) : 0;

    Serial.printf("[TX] ACK from %c seq=%u  RSSI=%d dBm  RTT=%lums",
                  src, ack_seq, (int)g_last_rssi, (unsigned long)rtt);
    if (g_mesh_on) Serial.printf("  mesh_src=0x%02X hops=%u", rxbuf[1], rxbuf[2]);
    Serial.println();

    /* Ignore stale ACK (wrong seq) */
    if (ack_seq != tx_seq) {
      Serial.printf("[TX] Stale ACK seq=%u (expected %u) — ignoring\n", ack_seq, tx_seq);
      rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
      return;
    }

    /* Verify data + count BER */
    bool match = (dl == data_len);
    u32 bit_errs = 0;
    if (dl > 0 && dl == data_len) {
      bit_errs = countBitErrors(&rxbuf[moff + PKT_HDR], &pkt[moff + PKT_HDR], dl);
      total_bits += dl * 8;
      error_bits += bit_errs;
      if (bit_errs > 0) match = false;
    } else if (dl != data_len) {
      match = false;
    }

    if (match) {
      ok_count++;
      lossRingPush(true);
    } else {
      corrupt_count++;
      lossRingPush(false);
      Serial.printf("    BER: %lu bit errors in %u bytes\n", (unsigned long)bit_errs, dl);
    }

    updateRSSIStats(g_last_rssi);
    throughput_bytes += data_len;
    retries = 0;
    consec_fail_rounds = 0;
    tx_seq++;                       /* advance seq on success */
    ledFlashGreen(30);

    updateOLED_TX();
    printCSVLine(rtt);

    /* Inter-packet gap: responsive to stop button */
    { u32 t0 = millis();
      while (millis() - t0 < 500) {
        feedLoop(); delay(5);
        Buttons bw = readButtons();
        if (bw.ok) { go_ready(); delay(2); drawStoppedScreen();
                     appState = APP_STOPPED; return; }
      }
    }
    send_test_packet();
  }
  else if (res == RF_RX_TIMEOUT) {
    retries++;
    fail_count++;
    lossRingPush(false);
    Serial.printf("[TX] No ACK (retry %d/%d) seq=%u\n", retries, MAX_RETRY, tx_seq);
    ledFlashRed(30);

    if (retries >= MAX_RETRY) {
      retries = 0;
      tx_seq++;                     /* advance seq — give up on this round */
      consec_fail_rounds++;

      /* ─── Mesh: route failure → broadcast re-discovery ─── */
      if (g_mesh_on && g_mesh_final_dest != 0xFF) {
        g_mesh_fail_cnt++;
        Serial.printf("[MESH] ACK fail %u/%u for dest 0x%02X\n",
                      g_mesh_fail_cnt, MESH_FAIL_LIMIT, g_mesh_final_dest);
        if (g_mesh_fail_cnt >= MESH_FAIL_LIMIT) {
          mesh_route_invalidate(g_mesh_final_dest);
          g_mesh_fail_cnt = 0;
          consec_fail_rounds = 0;  /* give broadcast discovery a fresh chance */
          Serial.println("[MESH] Route failed → switching to broadcast re-discovery");
        }
      }

      Serial.printf("[TX] Round failed — consec %d/%d\n",
                    consec_fail_rounds, MAX_CONSEC_FAIL);
      if (consec_fail_rounds >= MAX_CONSEC_FAIL) {
        go_ready(); ledRed();
        Serial.printf("[TX] CONNECTION LOST — RSSI=%d\n", (int)g_last_rssi);
        drawCatLost(g_last_rssi);
        appState = APP_CONN_LOST; return;
      }
      g_last_rssi = -128;
      updateOLED_TX();
      { u32 t0 = millis();
        while (millis() - t0 < 300) {
          feedLoop(); delay(5);
          Buttons bw = readButtons();
          if (bw.ok) { go_ready(); delay(2); drawStoppedScreen();
                       appState = APP_STOPPED; return; }
        }
      }
      send_test_packet();
    } else {
      /* Retry same seq — no increment */
      { u16 total = (g_mesh_on ? MESH_HDR : 0) + PKT_HDR + data_len;
        rf_start_tx(pkt, total, tx_timeout_ms);
      }
    }
  }
  else if (res == RF_TX_TIMEOUT || res == RF_ERROR) {
    fail_count++;
    lossRingPush(false);
    ledFlashRed(40);
    radio_init(cfg_rateIdx);
    updateOLED_TX();
    send_test_packet();
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ RX MODE                                                ███
   ═══════════════════════════════════════════════════════════════ */

static void updateOLED_RX() {
  if (millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  if (!_oledOK) return;

  u8g2.clearBuffer();
  char buf[22];

  u8g2.setFont(u8g2_font_7x13B_tr);
  snprintf(buf, sizeof(buf), "RX %s %uB %sdB", rate_labels[cfg_rateIdx], data_len, pwr_labels[cfg_pwrIdx]);
  u8g2.drawStr(0, 12, buf);

  u8g2.setFont(u8g2_font_6x10_tr);
  /* Line 2: packets + errors + dups  E=bit errors  D=duplicates(retries) */
  if (rx_err_count > 0 || dup_count > 0) {
    snprintf(buf, sizeof(buf), "Pk:%lu E:%lu D:%lu",
             (unsigned long)pktCount, (unsigned long)rx_err_count,
             (unsigned long)dup_count);
  } else {
    snprintf(buf, sizeof(buf), "Pkts:%lu", (unsigned long)pktCount);
  }
  u8g2.drawStr(0, 24, buf);

  /* Line 3: RSSI + margin */
  int16_t margin = g_last_rssi - NOISE_FLOOR_DBM;
  snprintf(buf, sizeof(buf), "RSSI:%d M:%ddB %s",
           (int)g_last_rssi, (int)margin, rssiQ(g_last_rssi));
  u8g2.drawStr(0, 35, buf);

  /* Line 4: RSSI best/worst/avg */
  if (rssi_cnt > 0) {
    int16_t avg = (int16_t)(rssi_sum / (int32_t)rssi_cnt);
    snprintf(buf, sizeof(buf), "B:%d W:%d A:%d",
             (int)rssi_best, (int)rssi_worst, (int)avg);
    u8g2.drawStr(0, 46, buf);
  } else {
    u8g2.drawStr(0, 46, "Listening...");
  }

  drawRSSIBar(g_last_rssi);
  u8g2.sendBuffer();
}

static void rx_loop() {
  feedLoop();

  /* Stop button */
  { Buttons b = readButtons();
    if (b.ok) {
      go_ready(); delay(2);
      Serial.println("[RX] STOPPED by user");
      drawStoppedScreen();
      appState = APP_STOPPED; delay(100); return;
    }
  }

  /* RX connection-lost timeout */
  if (pktCount > 0 && rx_last_pkt_ms > 0 &&
      millis() - rx_last_pkt_ms > RX_CONN_LOST_MS) {
    go_ready(); ledRed();
    Serial.printf("[RX] CONNECTION LOST — no pkts %lus — RSSI=%d\n",
                  (unsigned long)(RX_CONN_LOST_MS/1000), (int)g_last_rssi);
    drawCatLost(g_last_rssi);
    appState = APP_CONN_LOST; return;
  }

  RFResult res = rf_process();

  if (res == RF_RX_DONE) {
    rx_last_pkt_ms = millis();

    /* ─── Mesh relay processing ─── */
    if (g_mesh_on && g_rxd_len >= MESH_HDR) {
      uint8_t m_dest = rxbuf[0], m_src = rxbuf[1];
      uint8_t m_hops = rxbuf[2], m_ttl = rxbuf[3], m_seq = rxbuf[4];

      /* Validate mesh header — reject corrupted packets (BER in header bytes) */
      if (!mesh_hdr_valid(m_dest, m_src, m_hops, m_ttl)) {
        Serial.printf("[MESH] BAD HEADER: dest=0x%02X src=0x%02X hops=%u ttl=%u — dropped\n",
                      m_dest, m_src, m_hops, m_ttl);
        rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
        return;  /* discard entire packet — can't trust any field */
      }

      /* Un-blacklist radio_src — any received packet proves it's alive */
      for (int bi = 0; bi < MESH_BLACKLIST_MAX; bi++) {
        if (g_blacklist[bi].active && g_blacklist[bi].addr == g_rx_src_addr) {
          g_blacklist[bi].active = false;
          Serial.printf("[MESH] Un-blacklisted 0x%02X (alive — heard packet)\n", g_rx_src_addr);
        }
      }
      /* Passive route learning from every received packet (with RSSI quality) */
      mesh_route_update(m_src, g_rx_src_addr, m_hops + 1, g_last_rssi);
      if (g_rx_src_addr != m_src) mesh_route_update(g_rx_src_addr, g_rx_src_addr, 1, g_last_rssi);

      bool is_for_me = (m_dest == g_node_addr);
      bool is_bcast  = (m_dest == 0xFF || m_dest == 0x00);
      bool need_relay = (m_ttl > 0 && !mesh_is_seen(m_src, m_seq)
                         && (is_bcast || !is_for_me));

      if (need_relay) {
        mesh_mark_seen(m_src, m_seq);
        g_relay_len = g_rxd_len;
        memcpy(g_relay_buf, rxbuf, g_rxd_len);
        g_relay_buf[2]++;  g_relay_buf[3]--;        /* hops++, ttl-- */
        g_relay_pending = true;
      }

      if (is_for_me){
        /* ── Unicast to me: process + ACK ── */
        u16 off  = MESH_HDR;
        u16 dlen = (g_rxd_len > off + PKT_HDR) ? (g_rxd_len - off - PKT_HDR) : 0;
        u8 seq   = rxbuf[off + 1];
        bool is_dup = (seq == rx_last_seq);  rx_last_seq = seq;

        if (is_dup) {
          dup_count++;
          Serial.printf("[MESH-RX] DUP from 0x%02X seq=%u\n", m_src, seq);
        } else {
          pktCount++;  throughput_bytes += dlen;
          Serial.printf("[MESH-RX] #%lu from 0x%02X seq=%u hops=%u RSSI=%d %uB\n",
                        (unsigned long)pktCount, m_src, seq, m_hops,
                        (int)g_last_rssi, dlen);
          /* BER on data pattern */
          if (dlen > 0) {
            u32 be = 0;
            for (u16 i = 0; i < dlen; i++) {
              u8 x = rxbuf[off + PKT_HDR + i] ^ (u8)(i & 0xFF);
              while (x) { be += (x & 1); x >>= 1; }
            }
            total_bits += dlen * 8;  error_bits += be;
            if (be > 0) { rx_err_count++;
              Serial.printf("    BER: %lu bit errors in %u bytes\n", (unsigned long)be, dlen);
            }
          }
          updateRSSIStats(g_last_rssi);  printCSVLine(0);
        }

        /* Build ACK with mesh header back to sender */
        mesh_set_radio_dest(m_src);
        txData[0] = m_src;                  /* mesh dest = original sender */
        txData[1] = g_node_addr;            /* mesh src = me               */
        txData[2] = 1;                      /* fresh hop_count             */
        txData[3] = MESH_MAX_TTL;           /* fresh TTL                   */
        txData[4] = g_mesh_tx_seq++;        /* new mesh seq                */
        mesh_mark_seen(g_node_addr, txData[4]);
        txData[off + 0] = (u8)my_id;
        txData[off + 1] = seq;
        if (dlen > 0) memcpy(&txData[off + PKT_HDR], &rxbuf[off + PKT_HDR], dlen);

        ledFlashBlue(30);
        rf_start_tx(txData, (u16)(MESH_HDR + PKT_HDR + dlen), tx_timeout_ms);
        return;
      }

      /* ── Broadcast or not-for-me: relay only (no ACK) ── */
      if (is_bcast) {
        /* Process broadcast for stats */
        u16 dlen = (g_rxd_len > MESH_HDR + PKT_HDR) ?
                    (g_rxd_len - MESH_HDR - PKT_HDR) : 0;
        pktCount++;
        Serial.printf("[MESH-RX] BCAST from 0x%02X mseq=%u hops=%u RSSI=%d\n",
                      m_src, m_seq, m_hops, (int)g_last_rssi);
        updateRSSIStats(g_last_rssi);
      } else {
        Serial.printf("[MESH-RELAY] 0x%02X→0x%02X via 0x%02X hops=%u ttl=%u\n",
                      m_src, m_dest, g_node_addr, m_hops + 1, m_ttl - 1);
      }

      if (g_relay_pending) {
        mesh_set_radio_dest(rxbuf[0]);  /* route for original mesh_dest */
        delay(MESH_RELAY_DELAY);
        ledFlashBlue(20);
        rf_start_tx(g_relay_buf, g_relay_len, tx_timeout_ms);
        g_relay_pending = false;
      } else {
        rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
      }
      return;
    }

    /* ─── Non-mesh RX (original logic) ─── */
    char dest = (char)rxbuf[0];
    u8 seq    = rxbuf[1];
    u16 dlen  = (g_rxd_len > PKT_HDR) ? (g_rxd_len - PKT_HDR) : 0;

    bool is_dup = (seq == rx_last_seq);
    rx_last_seq = seq;

    if (is_dup) {
      dup_count++;
      Serial.printf("[RX] DUP seq=%u  RSSI=%d dBm (retry from TX)\n",
                    seq, (int)g_last_rssi);
    } else {
      pktCount++;
      throughput_bytes += g_rxd_len;
      Serial.printf("[RX] #%lu seq=%u  RSSI=%d dBm  %u bytes",
                    (unsigned long)pktCount, seq, (int)g_last_rssi, dlen);
      if (g_addr_enabled)
        Serial.printf("  from=0x%02X", g_rx_src_addr);
      Serial.println();

      if (dlen > 0) {
        u32 bit_errs = 0;
        for (u16 i = 0; i < dlen; i++) {
          u8 expected = (u8)(i & 0xFF);
          u8 x = rxbuf[PKT_HDR + i] ^ expected;
          while (x) { bit_errs += (x & 1); x >>= 1; }
        }
        total_bits += dlen * 8;
        error_bits += bit_errs;
        if (bit_errs > 0) {
          rx_err_count++;
          Serial.printf("    BER: %lu bit errors in %u bytes\n", (unsigned long)bit_errs, dlen);
        }
      }

      updateRSSIStats(g_last_rssi);
      printCSVLine(0);
    }

    if (dest == my_id) {
      txData[0] = (u8)my_id;
      txData[1] = seq;
      if (dlen > 0) memcpy(&txData[PKT_HDR], &rxbuf[PKT_HDR], dlen);
      ledFlashBlue(30);
      rf_start_tx(txData, (u16)(PKT_HDR + dlen), tx_timeout_ms);
    } else {
      ledFlashRed(30);
      updateOLED_RX();
      rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
    }
  }
  else if (res == RF_TX_DONE) {
    if (g_mesh_on)
      Serial.println("[MESH] TX done (ACK or relay)");
    else
      Serial.println("[RX] ACK sent");
    ledFlashGreen(30);
    updateOLED_RX();
    rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
  }
  else if (res == RF_TX_TIMEOUT || res == RF_ERROR) {
    ledFlashRed(40);
    if (res == RF_ERROR) radio_init(cfg_rateIdx);
    rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
  }
  else {
    /* Idle / waiting — keep display alive */
    updateOLED_RX();
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ TRX MODE (Ping-Pong — both sides TX + RX)             ███
   ═══════════════════════════════════════════════════════════════ */

static bool trx_is_tx_turn = false;    /* alternates each round */
static u32  trx_rtt        = 0;

static void updateOLED_TRX() {
  if (millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  if (!_oledOK) return;

  u8g2.clearBuffer();
  char buf[22];
  u8g2.setFont(u8g2_font_7x13B_tr);
  snprintf(buf, sizeof(buf), "TRX %s %uB %sdB", rate_labels[cfg_rateIdx], data_len, pwr_labels[cfg_pwrIdx]);
  u8g2.drawStr(0, 12, buf);
  u8g2.setFont(u8g2_font_6x10_tr);

  snprintf(buf, sizeof(buf), "Tx:%lu Rx:%lu",
           (unsigned long)round_num, (unsigned long)pktCount);
  u8g2.drawStr(0, 24, buf);

  int16_t margin = g_last_rssi - NOISE_FLOOR_DBM;
  snprintf(buf, sizeof(buf), "RSSI:%d M:%d %s",
           (int)g_last_rssi, (int)margin, rssiQ(g_last_rssi));
  u8g2.drawStr(0, 35, buf);

  float ber = total_bits > 0 ? (float)error_bits/(float)total_bits : 0;
  snprintf(buf, sizeof(buf), "BER:%.1e RTT:%lu",
           ber, (unsigned long)trx_rtt);
  u8g2.drawStr(0, 46, buf);

  drawRSSIBar(g_last_rssi);
  u8g2.sendBuffer();
}

static void trx_loop() {
  feedLoop();

  /* Stop button */
  { Buttons b = readButtons();
    if (b.ok) {
      go_ready(); Serial.println("[TRX] STOPPED");
      drawStoppedScreen(); appState = APP_STOPPED; delay(100); return;
    }
  }

  RFResult res = rf_process();

  if (trx_is_tx_turn) {
    /* TX phase — send packet, wait for response */
    if (res == RF_TX_DONE) {
      sent_at = millis();
      rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
    }
    else if (res == RF_RX_DONE) {
      trx_rtt = millis() - sent_at;
      u8 ack_seq = rxbuf[1];
      u16 dl = (g_rxd_len > PKT_HDR) ? (g_rxd_len - PKT_HDR) : 0;

      if (ack_seq != tx_seq) {
        Serial.printf("[TRX-TX] Stale ACK seq=%u (expected %u)\n", ack_seq, tx_seq);
        rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
        return;
      }

      ok_count++;
      updateRSSIStats(g_last_rssi);
      throughput_bytes += dl;
      if (dl > 0) {
        u32 be = countBitErrors(&rxbuf[PKT_HDR], &pkt[PKT_HDR], dl > data_len ? data_len : dl);
        total_bits += dl * 8; error_bits += be;
      }
      lossRingPush(true);
      tx_seq++;
      ledFlashGreen(30);
      updateOLED_TRX(); printCSVLine(trx_rtt);

      /* Switch to RX turn — listen for other side's packet */
      trx_is_tx_turn = false;
      { u32 t0 = millis();
        while (millis() - t0 < 300) {
          feedLoop(); delay(5);
          Buttons bw = readButtons();
          if (bw.ok) { go_ready(); drawStoppedScreen();
                       appState = APP_STOPPED; return; }
        }
      }
      rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms * 2);
    }
    else if (res == RF_RX_TIMEOUT) {
      fail_count++; lossRingPush(false);
      tx_seq++;                         /* give up this round */
      ledFlashRed(30);
      updateOLED_TRX();
      /* Retry TX with new seq (new round) */
      send_test_packet();
    }
  } else {
    /* RX phase — listen, then respond */
    if (res == RF_RX_DONE) {
      u8 seq = rxbuf[1];
      u16 dl = (g_rxd_len > PKT_HDR) ? (g_rxd_len - PKT_HDR) : 0;
      bool is_dup = (seq == rx_last_seq);
      rx_last_seq = seq;

      if (is_dup) {
        dup_count++;
        Serial.printf("[TRX-RX] DUP seq=%u\n", seq);
      } else {
        pktCount++;
        updateRSSIStats(g_last_rssi);
        throughput_bytes += dl;
        if (dl > 0) {
          u32 be = 0;
          for (u16 i = 0; i < dl; i++) {
            u8 x = rxbuf[PKT_HDR+i] ^ (u8)(i & 0xFF);
            while (x) { be += (x & 1); x >>= 1; }
          }
          total_bits += dl * 8; error_bits += be;
        }
      }
      ledFlashBlue(30);
      /* Echo back as response — include seq */
      txData[0] = (u8)my_id;
      txData[1] = seq;
      if (dl > 0) memcpy(&txData[PKT_HDR], &rxbuf[PKT_HDR], dl);
      rf_start_tx(txData, (u16)(PKT_HDR + dl), tx_timeout_ms);
    }
    else if (res == RF_TX_DONE) {
      /* Response sent — switch to TX turn */
      ledFlashGreen(30);
      updateOLED_TRX();
      trx_is_tx_turn = true;
      { u32 t0 = millis();
        while (millis() - t0 < 300) {
          feedLoop(); delay(5);
          Buttons bw = readButtons();
          if (bw.ok) { go_ready(); drawStoppedScreen();
                       appState = APP_STOPPED; return; }
        }
      }
      send_test_packet();
    }
    else if (res == RF_RX_TIMEOUT) {
      /* Nobody sent — I'll start TX */
      trx_is_tx_turn = true;
      send_test_packet();
    }
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ PER 1000 MODE (fixed packet test)                      ███
   ═══════════════════════════════════════════════════════════════ */

static void updateOLED_PER() {
  if (millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  if (!_oledOK) return;

  u8g2.clearBuffer();
  char buf[22];
  u8g2.setFont(u8g2_font_7x13B_tr);
  if (cfg_mode == MODE_TX || round_num > 0)
    snprintf(buf, sizeof(buf), "PER TX %lu/%lu", (unsigned long)round_num, (unsigned long)per_target);
  else
    snprintf(buf, sizeof(buf), "PER RX %lu", (unsigned long)pktCount);
  u8g2.drawStr(0, 12, buf);

  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "Ok:%lu Fail:%lu",
           (unsigned long)ok_count, (unsigned long)(fail_count+corrupt_count));
  u8g2.drawStr(0, 24, buf);

  float per = round_num > 0 ?
    100.0f * (float)(fail_count+corrupt_count) / (float)round_num : 0;
  snprintf(buf, sizeof(buf), "PER:%.2f%%", per);
  u8g2.drawStr(0, 35, buf);

  snprintf(buf, sizeof(buf), "RSSI:%d dBm", (int)g_last_rssi);
  u8g2.drawStr(0, 46, buf);

  /* Progress bar */
  u32 progress = round_num > 0 ? round_num : pktCount;
  int pw = (int)((progress * 118) / per_target);
  if (pw > 118) pw = 118;
  u8g2.drawFrame(4, 54, 120, 10);
  if (pw > 0) u8g2.drawBox(5, 55, pw, 8);
  u8g2.sendBuffer();
}

static void per_complete() {
  if (!_oledOK) return;
  u8g2.clearBuffer();
  char buf[22];
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 12, "PER COMPLETE");
  u8g2.setFont(u8g2_font_6x10_tr);

  float per = round_num > 0 ?
    100.0f * (float)(fail_count+corrupt_count) / (float)round_num : 0;
  snprintf(buf, sizeof(buf), "PER: %.3f%%", per);
  u8g2.drawStr(0, 26, buf);

  snprintf(buf, sizeof(buf), "%lu/%lu ok", (unsigned long)ok_count, (unsigned long)round_num);
  u8g2.drawStr(0, 38, buf);

  float ber = total_bits > 0 ? (float)error_bits/(float)total_bits : 0;
  snprintf(buf, sizeof(buf), "BER: %.2e", ber);
  u8g2.drawStr(0, 50, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(14, 63, "Press OK for menu");
  u8g2.sendBuffer();

  Serial.printf("[PER] Complete: %lu/%lu ok  PER=%.3f%%  BER=%.2e\n",
                (unsigned long)ok_count, (unsigned long)round_num, per, ber);
}

static void per_tx_loop() {
  feedLoop();

  /* Check if test complete */
  if (round_num >= per_target) {
    go_ready();
    per_complete();
    appState = APP_STOPPED;
    return;
  }

  /* Stop button */
  { Buttons b = readButtons();
    if (b.ok) {
      go_ready(); per_complete();
      appState = APP_STOPPED; delay(100); return;
    }
  }

  RFResult res = rf_process();

  if (res == RF_TX_DONE) {
    sent_at = millis();
    rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
  }
  else if (res == RF_RX_DONE) {
    u8 ack_seq = rxbuf[1];
    u16 dl = (g_rxd_len > PKT_HDR) ? (g_rxd_len - PKT_HDR) : 0;

    if (ack_seq != tx_seq) {
      Serial.printf("[PER] Stale ACK seq=%u (expected %u)\n", ack_seq, tx_seq);
      rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms);
      return;
    }

    bool match = (dl == data_len);
    if (dl > 0 && match) {
      u32 be = countBitErrors(&rxbuf[PKT_HDR], &pkt[PKT_HDR], dl);
      total_bits += dl * 8; error_bits += be;
      if (be > 0) match = false;
    }
    if (match) ok_count++;
    else corrupt_count++;
    updateRSSIStats(g_last_rssi);
    retries = 0;
    tx_seq++;
    ledFlashGreen(20);
    updateOLED_PER();
    send_test_packet();
  }
  else if (res == RF_RX_TIMEOUT) {
    retries++; fail_count++;
    if (retries >= MAX_RETRY) {
      retries = 0;
      tx_seq++;                     /* give up this round */
      send_test_packet();
    } else {
      /* Retry same seq */
      rf_start_tx(pkt, (u16)(PKT_HDR + data_len), tx_timeout_ms);
    }
    updateOLED_PER();
  }
  else if (res == RF_TX_TIMEOUT || res == RF_ERROR) {
    fail_count++;
    radio_init(cfg_rateIdx);
    send_test_packet();
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ CW CARRIER MODE (continuous rapid TX)                  ███
   ═══════════════════════════════════════════════════════════════
   NOTE: CMT2310A rev 0.8 disabled Direct TX mode.
   This mode sends packets continuously with minimum gap
   to approximate CW for antenna/spectrum analyzer testing.  */

static void updateOLED_CW() {
  if (millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  if (!_oledOK) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 12, "CW CARRIER TX");
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];
  snprintf(buf, sizeof(buf), "Rate: %s kbps", rate_labels[cfg_rateIdx]);
  u8g2.drawStr(0, 26, buf);
  snprintf(buf, sizeof(buf), "Pkts sent: %lu", (unsigned long)round_num);
  u8g2.drawStr(0, 38, buf);
  u8g2.drawStr(0, 50, "865 MHz +20dBm FSK");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(20, 63, "Press OK to stop");
  u8g2.sendBuffer();
}

static void cw_loop() {
  feedLoop();

  /* Stop button */
  { Buttons b = readButtons();
    if (b.ok) {
      go_ready(); Serial.println("[CW] STOPPED");
      drawStoppedScreen(); appState = APP_STOPPED; delay(100); return;
    }
  }

  RFResult res = rf_process();

  if (res == RF_TX_DONE || res == RF_IDLE) {
    /* Send next packet immediately — fill with 0xAA (alternating bits) */
    round_num++;
    memset(pkt, 0xAA, MAX_PAYLOAD);
    pkt[0] = 0xAA;
    rf_start_tx(pkt, 250, tx_timeout_ms);
    updateOLED_CW();
  }
  else if (res == RF_TX_TIMEOUT || res == RF_ERROR) {
    radio_init(cfg_rateIdx);
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ REGISTER DUMP MODE                                     ███
   ═══════════════════════════════════════════════════════════════ */

static void drawRegDump() {
  if (!_oledOK) return;

  /* Read key registers */
  u8 state     = chip_state();
  u8 rssi_val  = rr(CMT2310A_CTL_REG_34);
  u8 temp_val  = readChipTemp();
  u8 lbd_val   = rr(CMT2310A_CTL_REG_35);
  u8 fifo_cfg  = rr(CMT2310A_CTL_REG_19);
  u8 int_en    = rr(CMT2310A_CTL_REG_18);
  u8 pa_cfg    = rr(CMT2310A_CTL_REG_22);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];

  snprintf(buf, sizeof(buf), "State:  0x%02X", state);
  u8g2.drawStr(0, 10, buf);
  snprintf(buf, sizeof(buf), "RSSI:   %d dBm", (int)(int8_t)rssi_val);
  u8g2.drawStr(0, 20, buf);
  snprintf(buf, sizeof(buf), "Temp:   0x%02X", temp_val);
  u8g2.drawStr(0, 30, buf);
  snprintf(buf, sizeof(buf), "LBD:    0x%02X", lbd_val);
  u8g2.drawStr(0, 40, buf);
  snprintf(buf, sizeof(buf), "FIFO:0x%02X PA:0x%02X", fifo_cfg, pa_cfg);
  u8g2.drawStr(0, 50, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(14, 63, "Press OK for menu");
  u8g2.sendBuffer();

  /* Detailed Serial dump */
  Serial.println("\n=== REGISTER DUMP ===");
  Serial.printf("  CTL_REG_10 (State):    0x%02X\n", state);
  Serial.printf("  CTL_REG_34 (RSSI):     0x%02X = %d dBm\n", rssi_val, (int)(int8_t)rssi_val);
  Serial.printf("  CTL_REG_36 (Temp):     0x%02X\n", temp_val);
  Serial.printf("  CTL_REG_35 (LBD):      0x%02X\n", lbd_val);
  Serial.printf("  CTL_REG_19 (FIFO cfg): 0x%02X\n", fifo_cfg);
  Serial.printf("  CTL_REG_18 (INT en):   0x%02X\n", int_en);
  Serial.printf("  CTL_REG_22 (PA cfg):   0x%02X\n", pa_cfg);
  Serial.printf("  CTL_REG_04 (GPIO):     0x%02X\n", rr(CMT2310A_CTL_REG_04));
  Serial.printf("  CTL_REG_16 (INT1):     0x%02X\n", rr(CMT2310A_CTL_REG_16));
  Serial.printf("  CTL_REG_17 (INT2):     0x%02X\n", rr(CMT2310A_CTL_REG_17));

  /* Page1 TX config area dump */
  Serial.println("  Page1 TX area (0x10-0x27):");
  sel_page(1);
  Serial.print("    ");
  for (u8 i = 0x10; i <= 0x27; i++) {
    /* Read page1 config register via CRW port */
    u8 v = 0;
    spi_read(CMT2310A_CRW_PORT, &v, 1);
    /* Note: bulk read would be better but CRW port auto-increments */
  }
  sel_page(0);

  /* Read page1 RSSI config */
  sel_page(1);
  u8 rssi_cfg[2];
  /* Page1 registers 0x62 and 0x63 */
  spi_read(CMT2310A_CRW_PORT, rssi_cfg, 1);  /* simplified — may need address setup */
  sel_page(0);

  Serial.printf("  Rate: %s kbps\n", rate_labels[cfg_rateIdx]);
  Serial.println("========================");
}

/* ═══════════════════════════════════════════════════════════════
   ███ EEPROM LOG                                             ███
   ═══════════════════════════════════════════════════════════════
   Entry layout (16 bytes):
   [0]    mode (TX/RX/TRX/PER)
   [1]    rateIdx
   [2-3]  data_len
   [4-7]  total packets (round_num or pktCount)
   [8-11] ok_count
   [12]   rssi_best (int8)
   [13]   rssi_worst (int8)
   [14]   avg rssi (int8)
   [15]   checksum (XOR of 0-14)
   ═══════════════════════════════════════════════════════════════ */

static void saveLog() {
  uint16_t count = eepromReadU16(LOG_COUNT_ADDR);
  if (count == 0xFFFF) count = 0;
  if (count >= LOG_MAX_ENTRIES) count = 0;  /* wrap around */

  uint16_t addr = LOG_BASE_ADDR + count * LOG_ENTRY_SIZE;

  u8 entry[LOG_ENTRY_SIZE];
  entry[0] = cfg_mode;
  entry[1] = cfg_rateIdx;
  entry[2] = (data_len >> 8) & 0xFF;
  entry[3] = data_len & 0xFF;

  u32 total = round_num > 0 ? round_num : pktCount;
  entry[4] = (total >> 24) & 0xFF;
  entry[5] = (total >> 16) & 0xFF;
  entry[6] = (total >> 8) & 0xFF;
  entry[7] = total & 0xFF;

  entry[8]  = (ok_count >> 24) & 0xFF;
  entry[9]  = (ok_count >> 16) & 0xFF;
  entry[10] = (ok_count >> 8) & 0xFF;
  entry[11] = ok_count & 0xFF;

  entry[12] = (int8_t)rssi_best;
  entry[13] = (int8_t)rssi_worst;
  entry[14] = rssi_cnt > 0 ? (int8_t)(rssi_sum / (int32_t)rssi_cnt) : 0;

  /* Checksum */
  u8 chk = 0;
  for (int i = 0; i < 15; i++) chk ^= entry[i];
  entry[15] = chk;

  for (int i = 0; i < LOG_ENTRY_SIZE; i++) {
    eepromWrite(addr + i, entry[i]);
    kick();
  }

  count++;
  eepromWriteU16(LOG_COUNT_ADDR, count);
  Serial.printf("[log] Saved entry #%u to EEPROM addr 0x%04X\n", count, addr);
}

static void viewLogs() {
  uint16_t count = eepromReadU16(LOG_COUNT_ADDR);
  if (count == 0xFFFF || count == 0) {
    oledStatus("EEPROM Logs", "No logs saved", "", "", "Press OK for menu");
    Serial.println("[log] No logs found");
    return;
  }

  if (count > LOG_MAX_ENTRIES) count = LOG_MAX_ENTRIES;

  uint8_t viewIdx = 0;
  bool redraw = true;

  while (true) {
    feedLoop();

    if (redraw) {
      uint16_t addr = LOG_BASE_ADDR + viewIdx * LOG_ENTRY_SIZE;
      u8 entry[LOG_ENTRY_SIZE];
      for (int i = 0; i < LOG_ENTRY_SIZE; i++) entry[i] = eepromRead(addr + i);

      /* Verify checksum */
      u8 chk = 0;
      for (int i = 0; i < 15; i++) chk ^= entry[i];
      bool valid = (chk == entry[15]);

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7x13B_tr);
      char buf[22];
      snprintf(buf, sizeof(buf), "Log %u/%u %s",
               viewIdx + 1, count, valid ? "" : "ERR");
      u8g2.drawStr(0, 12, buf);

      if (valid) {
        u8g2.setFont(u8g2_font_6x10_tr);
        const char *m = entry[0] < MODE_COUNT ? mode_labels[entry[0]] : "?";
        const char *r = entry[1] < NUM_RATES ? rate_labels[entry[1]] : "?";
        uint16_t sz = ((uint16_t)entry[2] << 8) | entry[3];
        snprintf(buf, sizeof(buf), "%s %s %uB", m, r, sz);
        u8g2.drawStr(0, 24, buf);

        u32 total = ((u32)entry[4]<<24)|((u32)entry[5]<<16)|((u32)entry[6]<<8)|entry[7];
        u32 ok    = ((u32)entry[8]<<24)|((u32)entry[9]<<16)|((u32)entry[10]<<8)|entry[11];
        snprintf(buf, sizeof(buf), "Pkts:%lu Ok:%lu", (unsigned long)total, (unsigned long)ok);
        u8g2.drawStr(0, 36, buf);

        snprintf(buf, sizeof(buf), "RSSI B:%d W:%d A:%d",
                 (int)(int8_t)entry[12], (int)(int8_t)entry[13], (int)(int8_t)entry[14]);
        u8g2.drawStr(0, 48, buf);
      }

      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(1, 63, "L/R:nav  OK:menu");
      u8g2.sendBuffer();
      redraw = false;
    }

    Buttons b = readButtons();
    if (b.right && viewIdx < count - 1) { viewIdx++; redraw = true; }
    if (b.left  && viewIdx > 0)         { viewIdx--; redraw = true; }
    if (b.ok) return;
    delay(5);
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ AUTO RANGE TEST                                        ███
   ═══════════════════════════════════════════════════════════════
   Fully automated multi-rate, multi-size range test.
   Both nodes discover each other, then run 4 configs:
     {1.2kbps, 2.4kbps} × {128B, 250B}
   Collects: RSSI, RTT, BER, packet counts on both sides.
   Fail-safes: RSSI deviation detection, error rate restart.
   Results on OLED (scrollable) + Serial dump.
   ═══════════════════════════════════════════════════════════════ */

/* ─── AR packet types (pkt[1]) ─── */
#define AR_BEACON     0xA0
#define AR_BCN_ACK    0xA1
#define AR_CONFIG     0xB0
#define AR_CFG_ACK    0xB1
#define AR_SYNC       0xC0
#define AR_SYNC_ACK   0xC1
#define AR_TEST_PKT   0xD0
#define AR_TEST_ACK   0xD1
#define AR_RES_REQ    0xE0
#define AR_RES_RSP    0xE1

/* ─── AR test parameters (AR_MAX_CFG & ARRes are in ar_types.h) ─── */
#define AR_PKT_CNT   50      /* packets per config */
#define AR_MAX_RST   2       /* max restarts per config */

/* Presets — user selects on master via OLED menu */
#define AR_PRESET_SMALL  0    /* 32 + 64  (4 configs) */
#define AR_PRESET_LARGE  1    /* 128 + 250 (4 configs) */
#define AR_PRESET_ALL    2    /* 32 + 64 + 128 + 250 (8 configs) */
#ifdef DEV_SWEEP
static const char * const ar_preset_labels[3] = {
  "Sweep 64B", "Sweep 128B", "Sweep 128+250"
};
#else
static const char * const ar_preset_labels[3] = {
  "Quick 128B", "Full 128+250", "Speed test"
};
#endif

/* Runtime config table — populated by ar_setup_preset() on master,
   or incrementally from CONFIG packets on slave                      */
static uint8_t ar_num_cfg = 4;
static uint8_t ar_preset  = AR_PRESET_SMALL;  /* default: Quick 128B */
static uint8_t ar_rate[AR_MAX_CFG];
static uint8_t ar_size[AR_MAX_CFG];
static ARRes   ar_my[AR_MAX_CFG];       /* this node's stats */
static ARRes   ar_peer[AR_MAX_CFG];     /* other node's stats (received) */
static bool    ar_is_master = false;
static bool    ar_got_peer  = false;    /* did we receive peer results? */
static uint8_t ar_test_num  = 0;       /* sequential test number from EEPROM ring */

static void ar_setup_preset(uint8_t preset) {
  ar_preset = preset;
#ifdef DEV_SWEEP
  /* Deviation sweep: all 4 slots (0.6/2.4/3.6/5.0 kHz) */
  switch (preset) {
    case AR_PRESET_SMALL:   /* all 4 devs × 64B */
      ar_num_cfg = 4;
      ar_rate[0]=0; ar_size[0]=2;   /* D0.6 × 64B  */
      ar_rate[1]=1; ar_size[1]=2;   /* D2.4 × 64B  */
      ar_rate[2]=2; ar_size[2]=2;   /* D3.6 × 64B  */
      ar_rate[3]=3; ar_size[3]=2;   /* D5.0 × 64B  */
      break;
    case AR_PRESET_ALL:     /* all 4 devs × 128B + 250B */
      ar_num_cfg = 8;
      ar_rate[0]=0; ar_size[0]=3;   /* D0.6 × 128B */
      ar_rate[1]=0; ar_size[1]=4;   /* D0.6 × 250B */
      ar_rate[2]=1; ar_size[2]=3;   /* D2.4 × 128B */
      ar_rate[3]=1; ar_size[3]=4;   /* D2.4 × 250B */
      ar_rate[4]=2; ar_size[4]=3;   /* D3.6 × 128B */
      ar_rate[5]=2; ar_size[5]=4;   /* D3.6 × 250B */
      ar_rate[6]=3; ar_size[6]=3;   /* D5.0 × 128B */
      ar_rate[7]=3; ar_size[7]=4;   /* D5.0 × 250B */
      break;
    default:                /* LARGE: all 4 devs × 128B */
      ar_num_cfg = 4;
      ar_rate[0]=0; ar_size[0]=3;   /* D0.6 × 128B */
      ar_rate[1]=1; ar_size[1]=3;   /* D2.4 × 128B */
      ar_rate[2]=2; ar_size[2]=3;   /* D3.6 × 128B */
      ar_rate[3]=3; ar_size[3]=3;   /* D5.0 × 128B */
      break;
  }
#else
  switch (preset) {
    case AR_PRESET_SMALL:   /* Quick 128B — all 5 combos × 128B */
      ar_num_cfg = 5;
      ar_rate[0]=0; ar_size[0]=3;   /* 1.2/2.4 × 128B */
      ar_rate[1]=1; ar_size[1]=3;   /* 1.2/3.6 × 128B */
      ar_rate[2]=2; ar_size[2]=3;   /* 2.4/5   × 128B */
      ar_rate[3]=3; ar_size[3]=3;   /* 9.6k    × 128B */
      ar_rate[4]=4; ar_size[4]=3;   /* 19.2k   × 128B */
      break;
    case AR_PRESET_LARGE:   /* Full 128+250 — all 5 × 128B + top 3 × 250B */
      ar_num_cfg = 8;
      ar_rate[0]=0; ar_size[0]=3;   /* 1.2/2.4 × 128B */
      ar_rate[1]=1; ar_size[1]=3;   /* 1.2/3.6 × 128B */
      ar_rate[2]=2; ar_size[2]=3;   /* 2.4/5   × 128B */
      ar_rate[3]=3; ar_size[3]=3;   /* 9.6k    × 128B */
      ar_rate[4]=4; ar_size[4]=3;   /* 19.2k   × 128B */
      ar_rate[5]=2; ar_size[5]=4;   /* 2.4/5   × 250B */
      ar_rate[6]=3; ar_size[6]=4;   /* 9.6k    × 250B */
      ar_rate[7]=4; ar_size[7]=4;   /* 19.2k   × 250B */
      break;
    default:                /* Speed test (AR_PRESET_ALL=2) — 9.6k+19.2k × 128+250B */
      ar_num_cfg = 4;
      ar_rate[0]=3; ar_size[0]=3;   /* 9.6k  × 128B */
      ar_rate[1]=3; ar_size[1]=4;   /* 9.6k  × 250B */
      ar_rate[2]=4; ar_size[2]=3;   /* 19.2k × 128B */
      ar_rate[3]=4; ar_size[3]=4;   /* 19.2k × 250B */
      break;
  }
#endif
}

static void ar_res_init(ARRes *r) {
  memset(r, 0, sizeof(ARRes));
  r->rssi_min = 127;  r->rssi_max = -128;  r->rtt_min = 0xFFFFFFFF;
}

/* ─── Binary encode/decode helpers ─── */
static void pu16(u8 *p, uint16_t v) { p[0]=(u8)v; p[1]=(u8)(v>>8); }
static uint16_t gu16(const u8 *p) { return p[0]|((uint16_t)p[1]<<8); }
static void pi16(u8 *p, int16_t v) { pu16(p,(uint16_t)v); }
static int16_t gi16(const u8 *p) { return (int16_t)gu16(p); }
static void pu32(u8 *p, uint32_t v) { p[0]=(u8)v; p[1]=(u8)(v>>8); p[2]=(u8)(v>>16); p[3]=(u8)(v>>24); }
static uint32_t gu32(const u8 *p) { return p[0]|((u32)p[1]<<8)|((u32)p[2]<<16)|((u32)p[3]<<24); }

/* ─── Fail-safe: RSSI deviation ─── */
#define AR_RSSI_W 10
#define AR_RSSI_DEV 15       /* dBm deviation threshold */
static int16_t ar_rr[AR_RSSI_W];
static uint8_t ar_rri, ar_rrc, ar_rdev;
static uint32_t ar_err_ring;
static uint8_t  ar_err_cnt;

static void ar_fs_reset() {
  ar_rri = ar_rrc = ar_rdev = 0;
  ar_err_ring = 0; ar_err_cnt = 0;
  memset(ar_rr, 0, sizeof(ar_rr));
}

/* Returns true if RSSI anomaly → should restart */
static bool ar_chk_rssi(int16_t v) {
  if (ar_rrc >= AR_RSSI_W) {
    int32_t s = 0;
    for (uint8_t i = 0; i < AR_RSSI_W; i++) s += ar_rr[i];
    int16_t avg = (int16_t)(s / AR_RSSI_W);
    int16_t d = (v > avg) ? (v - avg) : (avg - v);
    if (d > AR_RSSI_DEV) { if (++ar_rdev >= 3) return true; }
    else ar_rdev = 0;
  }
  ar_rr[ar_rri] = v;
  ar_rri = (ar_rri + 1) % AR_RSSI_W;
  if (ar_rrc < AR_RSSI_W) ar_rrc++;
  return false;
}

/* Returns true if >50% errors in window of 20 → should restart */
static bool ar_chk_err(bool ok) {
  ar_err_ring = (ar_err_ring << 1) | (ok ? 0 : 1);
  if (ar_err_cnt < 20) ar_err_cnt++;
  if (!ok && ar_err_cnt >= 20) {
    uint8_t e = 0;  uint32_t m = ar_err_ring;
    for (uint8_t i = 0; i < 20; i++) { e += (m & 1); m >>= 1; }
    if (e > 10) return true;
  }
  return false;
}

/* ─── Blocking TX/RX wrappers around TRX state machine ─── */
static bool ar_tx_blk(u8 *buf, u16 len) {
  g_st = ST_IDLE;
  rf_start_tx(buf, len, tx_timeout_ms);
  u32 t0 = millis();
  while (millis() - t0 < tx_timeout_ms + 500) {   /* tight safety margin */
    feedLoop();
    RFResult r = rf_process();
    if (r == RF_TX_DONE) return true;
    if (r == RF_TX_TIMEOUT || r == RF_ERROR) return false;
    delay(1);
  }
  go_ready(); delay(2); g_st = ST_IDLE;
  return false;
}

static RFResult ar_rx_blk(u32 tmo) {
  g_st = ST_IDLE;
  rf_start_rx(rxbuf, MAX_PAYLOAD, tmo);
  u32 t0 = millis();
  while (millis() - t0 < tmo + 500) {             /* tight safety margin */
    feedLoop();
    RFResult r = rf_process();
    if (r != RF_BUSY && r != RF_IDLE) return r;
    delay(1);
  }
  go_ready(); delay(2); g_st = ST_IDLE;
  return RF_RX_TIMEOUT;
}

/* ─── Apply radio config ─── */
static void ar_apply_cfg(uint8_t ri, uint8_t si) {
  uint16_t sz = size_opts[si];
  g_2byte_len = (sz > 250);
  cfg_rateIdx = ri;  cfg_sizeIdx = si;  data_len = sz;
  ack_timeout_ms = calcACKTimeout();
  tx_timeout_ms  = calcTXTimeout();
  radio_init(cfg_rateIdx);
}

/* ─── OLED progress during test ─── */
static void ar_oled(uint8_t ci, uint16_t cur, uint16_t tot,
                    const char *ph, int16_t rssi) {
  if (!_oledOK || millis() - last_oled_ms < OLED_INTERVAL) return;
  last_oled_ms = millis();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 12, "Auto Range");
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[22];
  snprintf(buf, sizeof(buf), "%u/%u: %s %uB", ci + 1, ar_num_cfg,
           rate_labels[ar_rate[ci]], size_opts[ar_size[ci]]);
  u8g2.drawStr(0, 24, buf);
  snprintf(buf, sizeof(buf), "%s %u/%u", ph, cur, tot);
  u8g2.drawStr(0, 36, buf);
  snprintf(buf, sizeof(buf), "RSSI:%d dBm", (int)rssi);
  u8g2.drawStr(0, 48, buf);
  int pw = tot > 0 ? (int)((u32)cur * 118 / tot) : 0;
  if (pw > 118) pw = 118;
  u8g2.drawFrame(4, 54, 120, 10);
  if (pw > 0) u8g2.drawBox(5, 55, pw, 8);
  u8g2.sendBuffer();
}

/* ─── Discovery: both nodes find each other ─── */
static bool ar_discover() {
  /* Use 2.4kbps, small packets for fast discovery */
  g_2byte_len = false;  cfg_rateIdx = 2;  data_len = 16;
  ack_timeout_ms = 1000;  tx_timeout_ms = 300;
  /* Discovery uses broadcast so any node can respond */
  u8 saved_dest = g_tx_dest_addr;
  g_tx_dest_addr = 0x00;  /* broadcast for beacons */
  radio_init(cfg_rateIdx);

  Serial.printf("[AR-DISC] chipState=0x%02X  fifoFlags=0x%02X  rate=%s\n",
                chip_state(), rr(CMT2310A_CTL_REG_28), rate_labels[cfg_rateIdx]);

  u32 t0 = millis();
  const u32 disc_tmo = 60000;

  if (ar_is_master) {
    Serial.println("[AR] Master: searching for slave...");
    oledStatus("Auto Range", "Searching...", "Master (A)");
    for (int a = 0; a < 60 && millis() - t0 < disc_tmo; a++) {
      feedLoop();
      { Buttons b = readButtons(); if (b.ok) { g_tx_dest_addr = saved_dest; return false; } }
      pkt[0] = 0xFF;  pkt[1] = AR_BEACON;
      pkt[2] = 0xDE;  pkt[3] = 0xAD;  pkt[4] = (u8)my_id;
      bool txOk = ar_tx_blk(pkt, 5);
      Serial.printf("[AR-DISC] beacon #%d TX=%s  chipState=0x%02X\n",
                    a, txOk ? "OK" : "FAIL", chip_state());
      if (!txOk) { delay(200); continue; }
      RFResult r = ar_rx_blk(1500);
      Serial.printf("[AR-DISC] RX result=%d  len=%u", (int)r, g_rxd_len);
      if (r == RF_RX_DONE && g_rxd_len > 0) {
        Serial.printf("  data=[%02X %02X %02X %02X %02X]",
                      rxbuf[0], rxbuf[1],
                      g_rxd_len > 2 ? rxbuf[2] : 0,
                      g_rxd_len > 3 ? rxbuf[3] : 0,
                      g_rxd_len > 4 ? rxbuf[4] : 0);
      }
      Serial.println();
      if (r == RF_RX_DONE && g_rxd_len >= 5 &&
          rxbuf[1] == AR_BCN_ACK && rxbuf[2] == 0xDE && rxbuf[3] == 0xAD) {
        dest_id = (char)rxbuf[4];
        Serial.printf("[AR] Found slave '%c'  RSSI=%d\n", dest_id, (int)g_last_rssi);
        ledFlashGreen(300); return true;
      }
      if (a % 5 == 0) {
        char msg[22]; snprintf(msg, sizeof(msg), "Try %d/60...", a + 1);
        oledStatus("Auto Range", "Searching...", "Master (A)", msg);
      }
    }
    Serial.println("[AR] Discovery timeout");
    g_tx_dest_addr = saved_dest;  /* restore on failure */
    return false;
  } else {
    Serial.println("[AR] Slave: waiting for master...");
    oledStatus("Auto Range", "Waiting...", "Slave (B)");
    for (int a = 0; a < 60 && millis() - t0 < disc_tmo; a++) {
      feedLoop();
      { Buttons b = readButtons(); if (b.ok) { g_tx_dest_addr = saved_dest; return false; } }
      RFResult r = ar_rx_blk(2000);
      Serial.printf("[AR-DISC] slave RX #%d result=%d  len=%u", a, (int)r, g_rxd_len);
      if (r == RF_RX_DONE && g_rxd_len > 0) {
        Serial.printf("  data=[%02X %02X %02X %02X %02X]  RSSI=%d",
                      rxbuf[0], rxbuf[1],
                      g_rxd_len > 2 ? rxbuf[2] : 0,
                      g_rxd_len > 3 ? rxbuf[3] : 0,
                      g_rxd_len > 4 ? rxbuf[4] : 0,
                      (int)g_last_rssi);
      }
      Serial.println();
      if (r == RF_RX_DONE && g_rxd_len >= 5 &&
          rxbuf[1] == AR_BEACON && rxbuf[2] == 0xDE && rxbuf[3] == 0xAD) {
        dest_id = (char)rxbuf[4];
        /* g_tx_dest_addr was auto-swapped to master's radio addr by rf_process */
        pkt[0] = dest_id;  pkt[1] = AR_BCN_ACK;
        pkt[2] = 0xDE;  pkt[3] = 0xAD;  pkt[4] = (u8)my_id;
        bool ackOk = ar_tx_blk(pkt, 5);
        Serial.printf("[AR] Found master '%c'  RSSI=%d  ackTX=%s  radioAddr=0x%02X\n",
                      dest_id, (int)g_last_rssi, ackOk ? "OK" : "FAIL", g_rx_src_addr);
        ledFlashGreen(300); return true;
      }
      if (a % 5 == 0) {
        char msg[22]; snprintf(msg, sizeof(msg), "Try %d/60...", a + 1);
        oledStatus("Auto Range", "Waiting...", "Slave (B)", msg);
      }
    }
    Serial.println("[AR] Discovery timeout");
    g_tx_dest_addr = saved_dest;  /* restore on failure */
    return false;
  }
}

/* ─── Config switch: master sends CONFIG, slave ACKs, both switch ─── */
static bool ar_switch_cfg(uint8_t ci) {
  uint8_t ri = ar_rate[ci], si = ar_size[ci];
  Serial.printf("[AR] Switch to cfg %u: %s %uB\n", ci, rate_labels[ri], size_opts[si]);

  if (ar_is_master) {
    for (int retry = 0; retry < 5; retry++) {
      feedLoop();
      pkt[0] = dest_id;  pkt[1] = AR_CONFIG;
      pkt[2] = ri;  pkt[3] = si;
      pkt[4] = (u8)(AR_PKT_CNT & 0xFF);  pkt[5] = (u8)(AR_PKT_CNT >> 8);
      pkt[6] = ci;
      if (!ar_tx_blk(pkt, 7)) { delay(300); continue; }
      RFResult r = ar_rx_blk(2000);
      if (r == RF_RX_DONE && g_rxd_len >= 4 &&
          rxbuf[1] == AR_CFG_ACK && rxbuf[2] == ri && rxbuf[3] == si) {
        Serial.println("[AR] CONFIG ACK received");
        ar_apply_cfg(ri, si);
        delay(100);
        /* SYNC at new rate */
        for (int s = 0; s < 3; s++) {
          pkt[0] = dest_id;  pkt[1] = AR_SYNC;  pkt[2] = ci;
          if (ar_tx_blk(pkt, 3)) {
            RFResult sr = ar_rx_blk(ack_timeout_ms);
            if (sr == RF_RX_DONE && rxbuf[1] == AR_SYNC_ACK) {
              Serial.println("[AR] SYNC confirmed");
              return true;
            }
          }
          delay(200);
        }
        Serial.println("[AR] SYNC failed");
      }
      delay(500);
    }
    Serial.println("[AR] Config switch FAILED");
    return false;
  }
  /* Slave: handled in responder loop */
  return true;
}

/* ─── Master: run one config test (stop-and-wait with BER/RSSI/RTT) ─── */
static bool ar_master_test(uint8_t ci) {
  ARRes *res = &ar_my[ci];
  ar_res_init(res);
  ar_fs_reset();

  u16 pkt_len = (u16)(PKT_HDR + data_len);  /* same total as normal mode */
  u16 test_dl = (data_len > 1) ? (data_len - 1) : 0;
  u8 seq = 0;
  uint8_t consec_fail = 0;                   /* consecutive failures */
  #define AR_CONSEC_FAIL_MAX 10              /* abort config if peer seems offline */

  Serial.printf("[AR] Test cfg %u: %s %uB  %u pkts\n",
                ci, rate_labels[cfg_rateIdx], data_len, AR_PKT_CNT);

  for (uint16_t i = 0; i < AR_PKT_CNT; ) {
    feedLoop();
    { Buttons b = readButtons(); if (b.ok) return false; }

    /* Build test packet */
    pkt[0] = dest_id;  pkt[1] = AR_TEST_PKT;  pkt[2] = seq;
    for (u16 j = 0; j < test_dl; j++) pkt[3 + j] = (u8)(j & 0xFF);

    res->tx_sent++;
    u32 sent_t = millis();
    bool got_ack = false;

    for (u8 retry = 0; retry < 3 && !got_ack; retry++) {
      if (!ar_tx_blk(pkt, pkt_len)) { delay(50); continue; }
      RFResult r = ar_rx_blk(ack_timeout_ms);
      if (r == RF_RX_DONE && g_rxd_len >= 3 &&
          rxbuf[1] == AR_TEST_ACK && rxbuf[2] == seq) {
        got_ack = true;
      }
    }

    bool should_restart = false;

    if (got_ack) {
      consec_fail = 0;
      u32 rtt = millis() - sent_t;
      res->tx_ok++;
      res->rtt_sum += rtt;
      if (rtt < res->rtt_min) res->rtt_min = rtt;
      if (rtt > res->rtt_max) res->rtt_max = rtt;
      res->rtt_cnt++;

      /* BER check on echoed data */
      u16 rxdl = (g_rxd_len > 3) ? (g_rxd_len - 3) : 0;
      if (rxdl >= test_dl && test_dl > 0) {
        u32 be = 0;
        for (u16 j = 0; j < test_dl; j++) {
          u8 x = rxbuf[3 + j] ^ (u8)(j & 0xFF);
          while (x) { be += (x & 1); x >>= 1; }
        }
        res->total_bits += test_dl * 8;
        res->error_bits += be;
        if (be > 0) res->tx_corrupt++;
      }

      /* RSSI */
      res->rssi_sum += g_last_rssi;
      if (g_last_rssi < res->rssi_min) res->rssi_min = g_last_rssi;
      if (g_last_rssi > res->rssi_max) res->rssi_max = g_last_rssi;
      res->rssi_cnt++;

      if (ar_chk_rssi(g_last_rssi)) should_restart = true;
      ar_chk_err(true);
      ledFlashGreen(15);
    } else {
      res->tx_fail++;
      consec_fail++;
      if (ar_chk_err(false)) should_restart = true;
      ledFlashRed(15);
      /* Abort if peer seems offline */
      if (consec_fail >= AR_CONSEC_FAIL_MAX) {
        Serial.printf("[AR] Cfg %u: %u consecutive fails — peer offline?\n",
                      ci, consec_fail);
        oledStatus("Auto Range", "Peer OFFLINE?", "Skipping config...");
        delay(1500);
        res->valid = (res->tx_ok > 0);  /* partial results if any */
        return true;
      }
    }

    if (should_restart) {
      consec_fail = 0;
      res->restarts++;
      if (res->restarts > AR_MAX_RST) {
        Serial.printf("[AR] Cfg %u: max restarts reached\n", ci);
        res->valid = true;
        return true;
      }
      Serial.printf("[AR] Cfg %u: RESTART %u (rssi_dev=%u err_rate)\n",
                    ci, res->restarts, ar_rdev);
      uint8_t rst_bk = res->restarts;
      ar_res_init(res);
      res->restarts = rst_bk;
      ar_fs_reset();
      /* Re-send CONFIG so slave also resets its counters */
      if (!ar_switch_cfg(ci)) {
        Serial.printf("[AR] Cfg %u: restart sync failed\n", ci);
        res->valid = (res->tx_ok > 0);
        return true;
      }
      seq = 0;  i = 0;
      continue;
    }

    seq++;  i++;
    ar_oled(ci, i, AR_PKT_CNT, "TX", g_last_rssi);
    delay(30);  /* inter-packet gap */
  }

  res->valid = true;
  Serial.printf("[AR] Cfg %u done: ok=%u fail=%u corrupt=%u\n",
                ci, res->tx_ok, res->tx_fail, res->tx_corrupt);
  return true;
}

/* ─── Slave responder loop (reactive to master commands) ─── */
static void ar_slave_loop() {
  uint8_t cur_cfg = 0xFF;
  u8 last_seq = 0xFF;
  bool done = false;
  u32 idle_t = millis();
  uint8_t prev_rate = cfg_rateIdx;

  Serial.println("[AR-S] Entering responder loop");

  while (!done && millis() - idle_t < 120000) {  /* 2 min idle timeout */
    feedLoop();
    { Buttons b = readButtons();
      if (b.ok) { Serial.println("[AR-S] Cancelled"); return; }
    }

    RFResult r = ar_rx_blk(5000);
    if (r != RF_RX_DONE) continue;
    idle_t = millis();
    if (g_rxd_len < 2) continue;

    u8 type = rxbuf[1];

    /* ─── Stale beacon: re-ACK ─── */
    if (type == AR_BEACON && g_rxd_len >= 5) {
      pkt[0] = dest_id;  pkt[1] = AR_BCN_ACK;
      pkt[2] = 0xDE;  pkt[3] = 0xAD;  pkt[4] = (u8)my_id;
      ar_tx_blk(pkt, 5);
      continue;
    }

    /* ─── CONFIG: switch to new rate/size ─── */
    if (type == AR_CONFIG && g_rxd_len >= 7) {
      uint8_t ri = rxbuf[2], si = rxbuf[3], ci = rxbuf[6];
      Serial.printf("[AR-S] CONFIG: %s %uB cfg=%u\n",
                    rate_labels[ri], size_opts[si], ci);

      /* ACK at current rate */
      pkt[0] = dest_id;  pkt[1] = AR_CFG_ACK;
      pkt[2] = ri;  pkt[3] = si;
      ar_tx_blk(pkt, 4);

      /* Switch to new config */
      prev_rate = cfg_rateIdx;
      ar_apply_cfg(ri, si);

      /* Build slave-side config table from master's commands */
      if (ci < AR_MAX_CFG) {
        ar_rate[ci] = ri;  ar_size[ci] = si;
        if (ci + 1 > ar_num_cfg) ar_num_cfg = ci + 1;
      }

      /* Init stats for this config (ci=0xFE signals result-exchange switch) */
      cur_cfg = ci;
      if (cur_cfg < AR_MAX_CFG) {
        ar_res_init(&ar_my[cur_cfg]);
        last_seq = 0xFF;
      }

      /* Wait for SYNC at new rate */
      bool synced = false;
      delay(100);
      for (int s = 0; s < 5; s++) {
        RFResult sr = ar_rx_blk(3000);
        if (sr == RF_RX_DONE && rxbuf[1] == AR_SYNC) {
          pkt[0] = dest_id;  pkt[1] = AR_SYNC_ACK;
          ar_tx_blk(pkt, 2);
          synced = true;
          Serial.println("[AR-S] SYNC ACK sent");
          break;
        }
        /* Master might re-send CONFIG (our ACK lost) — re-ACK */
        if (sr == RF_RX_DONE && rxbuf[1] == AR_CONFIG) {
          pkt[0] = dest_id;  pkt[1] = AR_CFG_ACK;
          pkt[2] = rxbuf[2];  pkt[3] = rxbuf[3];
          ar_tx_blk(pkt, 4);
        }
      }
      if (!synced) {
        Serial.println("[AR-S] SYNC timeout — reverting");
        cfg_rateIdx = prev_rate;  g_2byte_len = false;  data_len = 16;
        radio_init(cfg_rateIdx);
      }
      continue;
    }

    /* ─── SYNC (late/retry): ACK it ─── */
    if (type == AR_SYNC) {
      pkt[0] = dest_id;  pkt[1] = AR_SYNC_ACK;
      ar_tx_blk(pkt, 2);
      continue;
    }

    /* ─── TEST packet: ACK + collect stats ─── */
    if (type == AR_TEST_PKT && g_rxd_len >= 3 && cur_cfg < AR_MAX_CFG) {
      u8 seq = rxbuf[2];
      ARRes *res = &ar_my[cur_cfg];
      bool is_dup = (seq == last_seq);
      last_seq = seq;

      if (!is_dup) {
        res->rx_recv++;
        /* BER check */
        u16 test_dl = (g_rxd_len > 3) ? (g_rxd_len - 3) : 0;
        u16 exp_dl = (data_len > 1) ? (data_len - 1) : 0;
        if (test_dl >= exp_dl && exp_dl > 0) {
          u32 be = 0;
          for (u16 j = 0; j < exp_dl; j++) {
            u8 x = rxbuf[3 + j] ^ (u8)(j & 0xFF);
            while (x) { be += (x & 1); x >>= 1; }
          }
          res->total_bits += exp_dl * 8;
          res->error_bits += be;
          if (be > 0) res->rx_err++;
        }
        /* RSSI */
        res->rssi_sum += g_last_rssi;
        if (g_last_rssi < res->rssi_min) res->rssi_min = g_last_rssi;
        if (g_last_rssi > res->rssi_max) res->rssi_max = g_last_rssi;
        res->rssi_cnt++;
      } else {
        res->rx_dup++;
      }

      /* Echo back as ACK */
      pkt[0] = dest_id;  pkt[1] = AR_TEST_ACK;  pkt[2] = seq;
      u16 echo_dl = (g_rxd_len > 3) ? (g_rxd_len - 3) : 0;
      if (echo_dl > 0) memcpy(&pkt[3], &rxbuf[3], echo_dl);
      ar_tx_blk(pkt, g_rxd_len);

      ledFlashBlue(15);
      if (!is_dup) {
        ar_oled(cur_cfg, res->rx_recv, AR_PKT_CNT, "RX", g_last_rssi);
        res->valid = true;
      }
      continue;
    }

    /* ─── RESULT REQUEST: send our stats ─── */
    if (type == AR_RES_REQ) {
      Serial.println("[AR-S] Sending results to master");
      /* Encode: 3B header + 20B × ar_num_cfg configs */
      pkt[0] = dest_id;  pkt[1] = AR_RES_RSP;  pkt[2] = ar_num_cfg;
      for (uint8_t c = 0; c < ar_num_cfg; c++) {
        u8 *p = &pkt[3 + c * 20];
        ARRes *r = &ar_my[c];
        pu16(p + 0, r->rx_recv);
        pu16(p + 2, r->rx_dup);
        pu16(p + 4, r->rx_err);
        pi16(p + 6, r->rssi_min);
        pi16(p + 8, r->rssi_max);
        int16_t avg = r->rssi_cnt > 0 ? (int16_t)(r->rssi_sum / r->rssi_cnt) : 0;
        pi16(p + 10, avg);
        pu32(p + 12, r->error_bits);
        pu32(p + 16, r->total_bits);
      }
      ar_tx_blk(pkt, 3 + ar_num_cfg * 20);
      done = true;
      continue;
    }
  }

  if (!done) {
    Serial.println("[AR-S] Idle timeout — master offline?");
    oledStatus("Auto Range", "Master OFFLINE?", "Test ended.", "", "Press OK");
  }
}

/* ─── Master: request and receive slave results ─── */
static void ar_get_peer_results() {
  ar_got_peer = false;
  for (int retry = 0; retry < 5; retry++) {
    feedLoop();
    pkt[0] = dest_id;  pkt[1] = AR_RES_REQ;
    if (!ar_tx_blk(pkt, 2)) { delay(300); continue; }
    RFResult r = ar_rx_blk(5000);
    if (r == RF_RX_DONE && g_rxd_len >= 3 && rxbuf[1] == AR_RES_RSP) {
      uint8_t ncfg = rxbuf[2];
      if (ncfg > ar_num_cfg) ncfg = ar_num_cfg;
      for (uint8_t c = 0; c < ncfg; c++) {
        const u8 *p = &rxbuf[3 + c * 20];
        ARRes *r2 = &ar_peer[c];
        memset(r2, 0, sizeof(ARRes));
        r2->rx_recv    = gu16(p + 0);
        r2->rx_dup     = gu16(p + 2);
        r2->rx_err     = gu16(p + 4);
        r2->rssi_min   = gi16(p + 6);
        r2->rssi_max   = gi16(p + 8);
        int16_t avg    = gi16(p + 10);
        r2->rssi_sum   = (int32_t)avg;  r2->rssi_cnt = 1;  /* store avg directly */
        r2->error_bits = gu32(p + 12);
        r2->total_bits = gu32(p + 16);
        r2->valid = true;
      }
      ar_got_peer = true;
      Serial.println("[AR] Received peer results");
      return;
    }
    delay(500);
  }
  Serial.println("[AR] Failed to get peer results");
}

/* ─── Serial dump results ─── */
/* RSSI quality label — typical sub-GHz FSK link thresholds */
static const char *ar_rssiQuality(int16_t r) {
  if (r >= -70) return "Excellent";
  if (r >= -85) return "Good";
  if (r >= -95) return "Fair";
  if (r >= -105) return "Weak";
  return "Very Weak";
}

/* BER quality label */
static const char *ar_berQuality(float ber_pct) {
  if (ber_pct <= 0.001f) return "Excellent";
  if (ber_pct <= 0.1f)   return "Good";
  if (ber_pct <= 1.0f)   return "Fair";
  if (ber_pct <= 5.0f)   return "Poor";
  return "Bad";
}

static void ar_serial_dump() {
  Serial.println("\n═══════════════════════════════════════════════════");
  Serial.println("         AUTO RANGE TEST RESULTS");
  Serial.println("═══════════════════════════════════════════════════");
  Serial.printf("Test #%u  |  Role: %s  |  Preset: %s  |  Configs: %u\n",
                ar_test_num,
                ar_is_master ? "MASTER" : "SLAVE",
                ar_preset < 3 ? ar_preset_labels[ar_preset] : "Custom",
                ar_num_cfg);
  Serial.println("───────────────────────────────────────────────────");

  int8_t best_cfg = -1, worst_cfg = -1;
  float best_ber = 1e9f, worst_ber = -1;

  for (uint8_t c = 0; c < ar_num_cfg; c++) {
    ARRes *my = &ar_my[c];
    Serial.printf("\nConfig %u: %s x %uB  [%s]  Restarts:%u\n",
                  c + 1, rate_labels[ar_rate[c]], size_opts[ar_size[c]],
                  my->valid ? "PASS" : "FAIL", my->restarts);

    float loss_pct = 0;
    if (ar_is_master) {
      loss_pct = my->tx_sent > 0 ? 100.0f * my->tx_fail / my->tx_sent : 0;
      u16 ack_clean = (my->tx_ok >= my->tx_corrupt) ? (my->tx_ok - my->tx_corrupt) : 0;
      Serial.printf("  TX: Sent=%u  Ok=%u  Fail=%u  AckErr=%u  Loss=%.1f%%  AckClean=%u\n",
                    my->tx_sent, my->tx_ok, my->tx_fail, my->tx_corrupt, loss_pct, ack_clean);
      if (my->rtt_cnt > 0) {
        u32 rtt_avg = my->rtt_sum / my->rtt_cnt;
        Serial.printf("  RTT: min=%lu  max=%lu  avg=%lu ms\n",
                      (unsigned long)my->rtt_min, (unsigned long)my->rtt_max,
                      (unsigned long)rtt_avg);
      }
    } else {
      loss_pct = AR_PKT_CNT > 0 ? 100.0f * (AR_PKT_CNT - my->rx_recv) / AR_PKT_CNT : 0;
      u16 rx_clean = (my->rx_recv >= my->rx_err) ? (my->rx_recv - my->rx_err) : 0;
      Serial.printf("  RX: Recv=%u  Dup=%u  Err=%u  Loss=%.1f%%  Clean=%u\n",
                    my->rx_recv, my->rx_dup, my->rx_err, loss_pct, rx_clean);
    }

    int16_t avg = 0;
    if (my->rssi_cnt > 0) {
      avg = (int16_t)(my->rssi_sum / my->rssi_cnt);
      Serial.printf("  RSSI: min=%d  max=%d  avg=%d dBm  [%s]\n",
                    (int)my->rssi_min, (int)my->rssi_max, (int)avg, ar_rssiQuality(avg));
    }
    float ber = my->total_bits > 0 ? (float)my->error_bits / (float)my->total_bits : 0;
    float ber_pct = ber * 100.0f;
    Serial.printf("  BER: %.4f%%  (%lu/%lu bits)  [%s]\n", ber_pct,
                  (unsigned long)my->error_bits, (unsigned long)my->total_bits,
                  ar_berQuality(ber_pct));

    if (my->valid && ber_pct < best_ber)  { best_ber = ber_pct;  best_cfg = c; }
    if (my->valid && ber_pct > worst_ber) { worst_ber = ber_pct; worst_cfg = c; }

    /* Peer results */
    if (ar_got_peer) {
      ARRes *pr = &ar_peer[c];
      u16 peer_clean = (pr->rx_recv >= pr->rx_err) ? (pr->rx_recv - pr->rx_err) : 0;
      Serial.printf("  Peer RX: Recv=%u  Dup=%u  Err=%u  Clean=%u\n",
                    pr->rx_recv, pr->rx_dup, pr->rx_err, peer_clean);
      if (pr->rssi_cnt > 0) {
        int16_t pavg = (int16_t)(pr->rssi_sum / pr->rssi_cnt);
        Serial.printf("  Peer RSSI: min=%d  max=%d  avg=%d dBm  [%s]\n",
                      (int)pr->rssi_min, (int)pr->rssi_max, (int)pavg, ar_rssiQuality(pavg));
      }
      float pber = pr->total_bits > 0 ? (float)pr->error_bits / (float)pr->total_bits : 0;
      Serial.printf("  Peer BER: %.4f%%\n", pber * 100.0f);
    }
    Serial.println("-----------------------------------------------------");
  }

  /* ─── Interpretation ─── */
  Serial.println("\n=== INTERPRETATION ===");
  if (best_cfg >= 0) {
    Serial.printf("  Best config:  #%d (%s x %uB) — lowest error rate (%.4f%% BER)\n",
                  best_cfg + 1, rate_labels[ar_rate[best_cfg]], size_opts[ar_size[best_cfg]], best_ber);
  }
  if (worst_cfg >= 0 && worst_cfg != best_cfg) {
    Serial.printf("  Weakest config: #%d (%s x %uB) — highest error rate (%.4f%% BER)\n",
                  worst_cfg + 1, rate_labels[ar_rate[worst_cfg]], size_opts[ar_size[worst_cfg]], worst_ber);
  }
  /* Overall link verdict from lowest-rate/weakest RSSI config (typically worst-case range) */
  for (uint8_t c = 0; c < ar_num_cfg; c++) {
    ARRes *my = &ar_my[c];
    if (!my->valid || my->rssi_cnt == 0) continue;
    int16_t avg = (int16_t)(my->rssi_sum / my->rssi_cnt);
    if (avg <= -95) {
      Serial.printf("  NOTE: Config #%u shows weak signal (%d dBm) — approaching range limit.\n",
                    c + 1, (int)avg);
    }
  }
  bool any_restart = false;
  for (uint8_t c = 0; c < ar_num_cfg; c++) if (ar_my[c].restarts > 0) any_restart = true;
  if (any_restart) {
    Serial.println("  NOTE: One or more configs needed restarts (RSSI jump or high error burst).");
  }
  if (ar_is_master) {
    bool any_corrupt = false;
    for (uint8_t c = 0; c < ar_num_cfg; c++) if (ar_my[c].tx_corrupt > 0) any_corrupt = true;
    if (any_corrupt) {
      Serial.println("  NOTE: Master 'Corrupt' counts are ACK-echo bit errors (return path),");
      Serial.println("        NOT errors in the original TX to slave. The slave may have");
      Serial.println("        received the data perfectly — check Peer/Slave BER to confirm.");
    }
  }
  if (!ar_got_peer) {
    Serial.println("  NOTE: Peer-side results unavailable (not exchanged or peer offline).");
  }
  Serial.println("=======================\n");

  /* CSV summary — prefixed with [AR-CSV] for easy parsing */
  Serial.printf("[AR-META] test=%u,role=%s,preset=%s,cfgs=%u\n",
                ar_test_num,
                ar_is_master ? "MASTER" : "SLAVE",
                ar_preset < 3 ? ar_preset_labels[ar_preset] : "Custom",
                ar_num_cfg);
  Serial.println("[AR-CSV] config,rate,size,tx_sent,tx_ok,tx_fail,ack_err,ack_clean,loss_pct,"
                 "rtt_avg,rssi_avg,ber_pct,peer_rx,peer_dup,peer_err,peer_clean,"
                 "peer_rssi_avg,peer_ber_pct");
  for (uint8_t c = 0; c < ar_num_cfg; c++) {
    ARRes *my = &ar_my[c];
    int16_t my_avg = my->rssi_cnt > 0 ? (int16_t)(my->rssi_sum / my->rssi_cnt) : 0;
    u32 rtt_a = my->rtt_cnt > 0 ? my->rtt_sum / my->rtt_cnt : 0;
    float ber_pct = (my->total_bits > 0 ? (float)my->error_bits / (float)my->total_bits : 0) * 100.0f;
    float loss_pct = ar_is_master
        ? (my->tx_sent > 0 ? 100.0f * my->tx_fail / my->tx_sent : 0)
        : (AR_PKT_CNT > 0 ? 100.0f * (AR_PKT_CNT - my->rx_recv) / AR_PKT_CNT : 0);
    u16 ack_clean = (my->tx_ok >= my->tx_corrupt) ? (my->tx_ok - my->tx_corrupt) : 0;
    ARRes *pr = ar_got_peer ? &ar_peer[c] : NULL;
    int16_t pr_avg = (pr && pr->rssi_cnt > 0) ? (int16_t)(pr->rssi_sum / pr->rssi_cnt) : 0;
    float pber_pct = (pr && pr->total_bits > 0 ? (float)pr->error_bits / (float)pr->total_bits : 0) * 100.0f;
    u16 peer_clean = (pr && pr->rx_recv >= pr->rx_err) ? (pr->rx_recv - pr->rx_err) : 0;
    Serial.printf("[AR-CSV] %u,%s,%u,%u,%u,%u,%u,%u,%.1f,%lu,%d,%.4f,%u,%u,%u,%u,%d,%.4f\n",
                  c + 1, rate_labels[ar_rate[c]], size_opts[ar_size[c]],
                  my->tx_sent, my->tx_ok, my->tx_fail, my->tx_corrupt, ack_clean, loss_pct,
                  (unsigned long)rtt_a, (int)my_avg, ber_pct,
                  pr ? pr->rx_recv : 0, pr ? pr->rx_dup : 0, pr ? pr->rx_err : 0, peer_clean,
                  (int)pr_avg, pber_pct);
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ AR EEPROM SAVE / LOAD / SERIAL RECALL                  ███
   ═══════════════════════════════════════════════════════════════
   Chip: 24CS512 (512 Kbit = 64KB) — plenty of room, so v3 packs
   every ARRes field losslessly (no ppm-compressed BER, no dropped
   rx_err counter, no reconstructed-from-average RTT/RSSI).

   "Latest" slot at 0x0400 (1024 bytes) — quick AR? access.
   History ring at 0x0800 — keeps last 10 tests. High-water mark
   ≈ 0x3010 (12.3KB), well inside the 64KB chip.

   Slot format (1024 bytes):
     +0x00  magic (0xAE, 0x03)
     +0x02  flags (1B: bit0=master, bit1=got_peer)
     +0x03  num_cfg (1B: 4 or 8)
     +0x04  test_number (2B LE)
     +0x06  preset_id (1B)
     +0x07  reserved (1B)
     +0x08  rate[8] (8B)
     +0x10  size_idx[8] (8B)
     +0x18  per-config data: num_cfg × 96B  (48B my + 48B peer)
   Max used: 24 + 8×96 = 792 bytes (232 bytes spare for future fields).

   Ring header at 0x0800 (16 bytes):
     +0  magic (0xAD, 0x10)
     +2  ring_head (1B: next write slot 0-9)
     +3  ring_used (1B: slots with data, max 10)
     +4  total_count (2B LE: total tests ever, for numbering)
     +6  reserved (10B)

   Packed ARRes (48 bytes, exact — no lossy reconstruction):
     +0  valid(1) +1 restarts(1)
     +2  tx_sent(2) +4 tx_ok(2) +6 tx_fail(2) +8 tx_corrupt/AckErr(2)
     +10 rtt_sum(4) +14 rtt_min(4) +18 rtt_max(4) +22 rtt_cnt(2)
     +24 rx_recv(2) +26 rx_dup(2) +28 rx_err(2)   <- was missing in v2
     +30 rssi_min(2) +32 rssi_max(2) +34 rssi_sum(4) +38 rssi_cnt(2)
     +40 total_bits(4) +44 error_bits(4)          <- exact, no ppm cap
   ─────────────────────────────────────────────────────────────── */
#define AR_EE_LATEST   0x0400          /* latest-test slot */
#define AR_EE_RING_HDR 0x0800          /* ring header (16B) */
#define AR_EE_RING_DAT 0x0810          /* first ring slot */
#define AR_EE_SLOT_SZ  1024
#define AR_EE_RING_MAX 10
#define AR_EE_MAGIC_0  0xAE
#define AR_EE_MAGIC_1  0x03            /* v3 format — lossless pack */
#define AR_EE_PER_CFG  48              /* bytes per ARRes (packed) */

/* Pack one ARRes into 48 bytes — every field stored exactly */
static void ar_ee_pack(const ARRes *r, u8 *p) {
  p[0] = r->valid ? 1 : 0;
  p[1] = r->restarts;
  pu16(p + 2,  r->tx_sent);
  pu16(p + 4,  r->tx_ok);
  pu16(p + 6,  r->tx_fail);
  pu16(p + 8,  r->tx_corrupt);                 /* AckErr */
  pu32(p + 10, r->rtt_sum);
  pu32(p + 14, (r->rtt_min == 0xFFFFFFFF) ? 0 : r->rtt_min);
  pu32(p + 18, r->rtt_max);
  pu16(p + 22, r->rtt_cnt);
  pu16(p + 24, r->rx_recv);
  pu16(p + 26, r->rx_dup);
  pu16(p + 28, r->rx_err);                     /* clean = rx_recv - rx_err */
  pi16(p + 30, r->rssi_min);
  pi16(p + 32, r->rssi_max);
  pu32(p + 34, (uint32_t)r->rssi_sum);
  pu16(p + 38, r->rssi_cnt);
  pu32(p + 40, r->total_bits);
  pu32(p + 44, r->error_bits);
}

/* Unpack 48 bytes → ARRes, exact round-trip */
static void ar_ee_unpack(const u8 *p, ARRes *r) {
  memset(r, 0, sizeof(ARRes));
  r->valid      = (p[0] != 0);
  r->restarts   = p[1];
  r->tx_sent    = gu16(p + 2);
  r->tx_ok      = gu16(p + 4);
  r->tx_fail    = gu16(p + 6);
  r->tx_corrupt = gu16(p + 8);
  r->rtt_sum    = gu32(p + 10);
  r->rtt_min    = gu32(p + 14);
  r->rtt_max    = gu32(p + 18);
  r->rtt_cnt    = gu16(p + 22);
  r->rx_recv    = gu16(p + 24);
  r->rx_dup     = gu16(p + 26);
  r->rx_err     = gu16(p + 28);
  r->rssi_min   = gi16(p + 30);
  r->rssi_max   = gi16(p + 32);
  r->rssi_sum   = (int32_t)gu32(p + 34);
  r->rssi_cnt   = gu16(p + 38);
  r->total_bits = gu32(p + 40);
  r->error_bits = gu32(p + 44);
}

/* ─── Write one slot at a given EEPROM address ─── */
static void ar_ee_write_slot(uint16_t base) {
  uint16_t addr = base;
  eepromWrite(addr++, AR_EE_MAGIC_0);
  eepromWrite(addr++, AR_EE_MAGIC_1);
  u8 flags = (ar_is_master ? 1 : 0) | (ar_got_peer ? 2 : 0);
  eepromWrite(addr++, flags);
  eepromWrite(addr++, ar_num_cfg);
  eepromWrite(addr++, (u8)(ar_test_num & 0xFF));
  eepromWrite(addr++, (u8)(ar_test_num >> 8));
  eepromWrite(addr++, ar_preset);
  eepromWrite(addr++, 0);  /* reserved */
  kick();
  /* rate[] and size[] arrays (8 bytes each) */
  for (uint8_t i = 0; i < AR_MAX_CFG; i++) eepromWrite(addr++, ar_rate[i]);
  for (uint8_t i = 0; i < AR_MAX_CFG; i++) eepromWrite(addr++, ar_size[i]);
  kick();
  /* Per-config data: my + peer */
  u8 buf[AR_EE_PER_CFG];
  for (uint8_t c = 0; c < ar_num_cfg; c++) {
    ar_ee_pack(&ar_my[c], buf);
    for (uint8_t i = 0; i < AR_EE_PER_CFG; i++) eepromWrite(addr++, buf[i]);
    kick();
    ar_ee_pack(&ar_peer[c], buf);
    for (uint8_t i = 0; i < AR_EE_PER_CFG; i++) eepromWrite(addr++, buf[i]);
    kick();
  }
  Serial.printf("[AR] Wrote slot @ 0x%04X (%u cfgs, %u bytes)\n",
                base, ar_num_cfg, (unsigned)(addr - base));
}

/* ─── Read one slot from EEPROM, populate globals ─── */
static bool ar_ee_read_slot(uint16_t base) {
  if (eepromRead(base) != AR_EE_MAGIC_0 ||
      eepromRead(base + 1) != AR_EE_MAGIC_1) return false;
  uint16_t addr = base + 2;
  u8 flags = eepromRead(addr++);
  ar_is_master = (flags & 1);
  ar_got_peer  = (flags & 2);
  ar_num_cfg   = eepromRead(addr++);
  if (ar_num_cfg > AR_MAX_CFG) ar_num_cfg = AR_MAX_CFG;
  ar_test_num  = eepromRead(addr) | ((uint16_t)eepromRead(addr + 1) << 8);
  addr += 2;
  ar_preset    = eepromRead(addr++);
  addr++;  /* skip reserved */
  /* rate[] and size[] */
  for (uint8_t i = 0; i < AR_MAX_CFG; i++) ar_rate[i] = eepromRead(addr++);
  for (uint8_t i = 0; i < AR_MAX_CFG; i++) ar_size[i] = eepromRead(addr++);
  /* Per-config data */
  u8 buf[AR_EE_PER_CFG];
  for (uint8_t c = 0; c < ar_num_cfg; c++) {
    for (uint8_t i = 0; i < AR_EE_PER_CFG; i++) buf[i] = eepromRead(addr++);
    ar_ee_unpack(buf, &ar_my[c]);
    for (uint8_t i = 0; i < AR_EE_PER_CFG; i++) buf[i] = eepromRead(addr++);
    ar_ee_unpack(buf, &ar_peer[c]);
  }
  return true;
}

/* ─── Ring header read/write ─── */
static void ar_ring_read(uint8_t *head, uint8_t *used, uint16_t *total) {
  if (eepromRead(AR_EE_RING_HDR) != 0xAD ||
      eepromRead(AR_EE_RING_HDR + 1) != 0x10) {
    *head = 0;  *used = 0;  *total = 0;
    return;
  }
  *head  = eepromRead(AR_EE_RING_HDR + 2);
  *used  = eepromRead(AR_EE_RING_HDR + 3);
  *total = eepromRead(AR_EE_RING_HDR + 4) |
           ((uint16_t)eepromRead(AR_EE_RING_HDR + 5) << 8);
  if (*head >= AR_EE_RING_MAX) *head = 0;
  if (*used > AR_EE_RING_MAX)  *used = AR_EE_RING_MAX;
}

static void ar_ring_write(uint8_t head, uint8_t used, uint16_t total) {
  eepromWrite(AR_EE_RING_HDR,     0xAD);
  eepromWrite(AR_EE_RING_HDR + 1, 0x10);
  eepromWrite(AR_EE_RING_HDR + 2, head);
  eepromWrite(AR_EE_RING_HDR + 3, used);
  eepromWrite(AR_EE_RING_HDR + 4, (u8)(total & 0xFF));
  eepromWrite(AR_EE_RING_HDR + 5, (u8)(total >> 8));
}

/* ─── Save: write to "latest" slot + push onto history ring ─── */
static void ar_ee_save() {
  Serial.println("[AR] Saving results to EEPROM...");

  /* Read ring state */
  uint8_t rh, ru; uint16_t rt;
  ar_ring_read(&rh, &ru, &rt);
  rt++;                                    /* increment total test count */
  ar_test_num = rt;

  /* 1. Write "latest" slot at 0x0400 */
  ar_ee_write_slot(AR_EE_LATEST);

  /* 2. Write to history ring */
  uint16_t slot_addr = AR_EE_RING_DAT + (uint16_t)rh * AR_EE_SLOT_SZ;
  ar_ee_write_slot(slot_addr);

  /* Advance ring pointer */
  rh = (rh + 1) % AR_EE_RING_MAX;
  if (ru < AR_EE_RING_MAX) ru++;
  ar_ring_write(rh, ru, rt);

  Serial.printf("[AR] Test #%u saved. History: %u/%u slots used.\n",
                rt, ru, AR_EE_RING_MAX);
}

/* ─── Load latest (for AR?) ─── */
static bool ar_ee_load() {
  return ar_ee_read_slot(AR_EE_LATEST);
}

/* ─── Load history slot by ring index (0 = most recent) ─── */
static bool ar_ee_load_history(uint8_t idx) {
  uint8_t rh, ru; uint16_t rt;
  ar_ring_read(&rh, &ru, &rt);
  if (idx >= ru) return false;
  /* Ring order: slot 0 = newest. Map to physical slot. */
  uint8_t phys = (rh + AR_EE_RING_MAX - 1 - idx) % AR_EE_RING_MAX;
  uint16_t addr = AR_EE_RING_DAT + (uint16_t)phys * AR_EE_SLOT_SZ;
  return ar_ee_read_slot(addr);
}

/* ─── List saved tests (one-line summary each) ─── */
static void ar_ee_list() {
  uint8_t rh, ru; uint16_t rt;
  ar_ring_read(&rh, &ru, &rt);
  if (ru == 0) {
    Serial.println("[AR] No test history. Run a test first.");
    return;
  }
  Serial.printf("\n=== Test History (%u/%u slots) ===\n", ru, AR_EE_RING_MAX);
  Serial.println(" Slot  Test#  Role    Preset        Cfgs  Status");
  Serial.println(" ----  -----  ------  ------------  ----  ------");
  for (uint8_t i = 0; i < ru; i++) {
    /* Save/restore globals — just peek at the slot header */
    uint8_t phys = (rh + AR_EE_RING_MAX - 1 - i) % AR_EE_RING_MAX;
    uint16_t base = AR_EE_RING_DAT + (uint16_t)phys * AR_EE_SLOT_SZ;
    if (eepromRead(base) != AR_EE_MAGIC_0) continue;
    u8 flags = eepromRead(base + 2);
    u8 ncfg  = eepromRead(base + 3);
    uint16_t tnum = eepromRead(base + 4) | ((uint16_t)eepromRead(base + 5) << 8);
    u8 preset = eepromRead(base + 6);
    const char *pname = (preset < 3) ? ar_preset_labels[preset] : "?";
    bool is_m = (flags & 1);
    /* Quick validity check: count valid configs */
    uint8_t valid_cnt = 0;
    uint16_t daddr = base + 0x18;  /* start of config data */
    for (uint8_t c = 0; c < ncfg && c < AR_MAX_CFG; c++) {
      if (eepromRead(daddr) != 0) valid_cnt++;  /* first byte of packed = valid flag */
      daddr += AR_EE_PER_CFG * 2;  /* skip my + peer */
    }
    Serial.printf("  %u     %3u   %-6s  %-12s  %u     %u/%u pass\n",
                  i + 1, tnum, is_m ? "MASTER" : "SLAVE", pname, ncfg, valid_cnt, ncfg);
    kick();
  }
  Serial.println("=================================\n");
  Serial.println("  Commands: AR? [latest]  AR? LIST  AR? 1..10  AR? ALL  AR? CLEAR\n");
}

/* Serial command handler — call from loop().
   Commands:
     AR?       — dump latest test results
     AR? LIST  — list all saved tests
     AR? N     — dump test #N from history (1=newest)
     AR? ALL   — dump all tests
     AR? CLEAR — clear test results only
     EE WIPE   — wipe entire EEPROM (event log + all tests)
     HELP / ?  — help text                                       */
static void ar_serial_cmd() {
  static char cmdbuf[32];
  static uint8_t cmdlen = 0;

  while (Serial.available()) {
    char ch = Serial.read();
    Serial.write(ch);                    /* echo */

    if (ch == '\n' || ch == '\r') {
      if (cmdlen == 0) { Serial.println(); continue; }
      cmdbuf[cmdlen] = '\0';
      Serial.println();

      /* Trim trailing spaces */
      while (cmdlen > 0 && cmdbuf[cmdlen-1] == ' ') cmdlen--;
      cmdbuf[cmdlen] = '\0';

      /* Convert to uppercase */
      for (uint8_t i = 0; i < cmdlen; i++) {
        if (cmdbuf[i] >= 'a' && cmdbuf[i] <= 'z') cmdbuf[i] -= 32;
      }

      Serial.printf("[cmd] Received: '%s'\n", cmdbuf);

      /* ─── Parse AR? family ─── */
      if (strcmp(cmdbuf, "AR?") == 0) {
        /* Latest test */
        Serial.println("\n[AR] Loading latest results...");
        if (ar_ee_load()) {
          Serial.printf("[AR] Test #%u loaded (%s, %s, %u cfgs)\n\n",
                        ar_test_num,
                        ar_is_master ? "MASTER" : "SLAVE",
                        ar_preset < 3 ? ar_preset_labels[ar_preset] : "?",
                        ar_num_cfg);
          ar_serial_dump();
        } else {
          Serial.println("[AR] No saved results. Run a test first.\n");
        }
      }
      else if (strncmp(cmdbuf, "AR? ", 4) == 0) {
        char *arg = cmdbuf + 4;
        while (*arg == ' ') arg++;

        if (strcmp(arg, "LIST") == 0) {
          ar_ee_list();
        }
        else if (strcmp(arg, "ALL") == 0) {
          uint8_t rh2, ru2; uint16_t rt2;
          ar_ring_read(&rh2, &ru2, &rt2);
          if (ru2 == 0) {
            Serial.println("[AR] No history.\n");
          } else {
            /* Save current globals */
            uint8_t sv_ncfg = ar_num_cfg; bool sv_master = ar_is_master;
            bool sv_peer = ar_got_peer; uint8_t sv_preset = ar_preset;
            uint16_t sv_tnum = ar_test_num;
            for (uint8_t i = 0; i < ru2; i++) {
              if (ar_ee_load_history(i)) {
                Serial.printf("\n╔══ History slot %u — Test #%u ══╗\n", i + 1, ar_test_num);
                ar_serial_dump();
              }
            }
            /* Restore globals from latest */
            ar_ee_load();
          }
        }
        else if (strcmp(arg, "CLEAR") == 0) {
          ar_ring_write(0, 0, 0);
          /* Also clear latest slot magic */
          eepromWrite(AR_EE_LATEST, 0xFF);
          eepromWrite(AR_EE_LATEST + 1, 0xFF);
          Serial.println("[AR] History and latest results cleared.\n");
        }
        else {
          /* AR? N — load Nth history slot (1-based) */
          int n = atoi(arg);
          if (n >= 1 && n <= AR_EE_RING_MAX) {
            if (ar_ee_load_history((uint8_t)(n - 1))) {
              Serial.printf("\n[AR] History slot %d — Test #%u\n\n", n, ar_test_num);
              ar_serial_dump();
              /* Restore latest */
              ar_ee_load();
            } else {
              Serial.printf("[AR] Slot %d is empty.\n\n", n);
            }
          } else {
            Serial.printf("[AR] Invalid slot: %s (use 1..%d)\n\n", arg, AR_EE_RING_MAX);
          }
        }
      }
      else if (strcmp(cmdbuf, "EE WIPE") == 0) {
        Serial.println("[EE] Wiping all EEPROM data...");
        /* 1. Clear event log count */
        eepromWriteU16(LOG_COUNT_ADDR, 0);
        /* 2. Invalidate AR latest slot */
        eepromWrite(AR_EE_LATEST, 0xFF);
        eepromWrite(AR_EE_LATEST + 1, 0xFF);
        /* 3. Zero AR ring header */
        for (uint8_t i = 0; i < 16; i++) eepromWrite(AR_EE_RING_HDR + i, 0);
        /* 4. Invalidate each ring slot's magic bytes */
        for (uint8_t s = 0; s < AR_EE_RING_MAX; s++) {
          uint16_t sa = AR_EE_RING_DAT + (uint16_t)s * AR_EE_SLOT_SZ;
          eepromWrite(sa, 0xFF);
          eepromWrite(sa + 1, 0xFF);
        }
        Serial.println("[EE] Done — event log + all test results wiped.\n");
      }
      /* ─── Mesh address commands ─── */
      else if (strcmp(cmdbuf, "MESH?") == 0) {
        Serial.println("\n=== Mesh Config ===");
        Serial.printf("  Address filtering: %s\n", g_addr_enabled ? "ON" : "OFF");
        Serial.printf("  Mesh relay:        %s (DIP6)\n", g_mesh_on ? "ON" : "OFF");
        Serial.printf("  FIFO debug:        %s\n", g_addr_debug ? "ON" : "OFF");
        Serial.printf("  Node address:      0x%02X (%u)\n", g_node_addr, g_node_addr);
        Serial.printf("  Radio TX dest:     0x%02X\n", g_tx_dest_addr);
        if (g_mesh_on) {
          Serial.printf("  Mesh final dest:   0x%02X\n", g_mesh_final_dest);
          Serial.printf("  Route fail cnt:    %u/%u\n", g_mesh_fail_cnt, MESH_FAIL_LIMIT);
          Serial.printf("  Route expiry:      %us\n", MESH_ROUTE_EXPIRY / 1000);
        }
        Serial.printf("  Last RX from:      0x%02X\n", g_rx_src_addr);
        Serial.printf("  Last RX dest:      0x%02X\n", g_rx_dest_addr);
        Serial.println("  HW broadcast:      0x00 and 0xFF (always accepted)");
        Serial.println("  (Serial cmds override DIP until next test start)");
        Serial.println("===================\n");
        if (g_mesh_on) mesh_route_print();
      }
      else if (strcmp(cmdbuf, "ROUTE?") == 0) {
        mesh_route_print();
      }
      else if (strcmp(cmdbuf, "ROUTE CLEAR") == 0) {
        mesh_route_clear();  /* also clears blacklist */
        Serial.println("[MESH] Route table + blacklist cleared.\n");
      }
      else if (strcmp(cmdbuf, "BLACKLIST?") == 0) {
        bool any = false;
        Serial.println("\n=== Next-Hop Blacklist ===");
        for (int i = 0; i < MESH_BLACKLIST_MAX; i++) {
          if (!g_blacklist[i].active) continue;
          any = true;
          u32 rem = 0;
          if (millis() < g_blacklist[i].until)
            rem = (g_blacklist[i].until - millis()) / 1000;
          Serial.printf("  0x%02X  unblocks in %lus\n",
                        g_blacklist[i].addr, (unsigned long)rem);
        }
        if (!any) Serial.println("  (none — all relays healthy)");
        Serial.println("==========================\n");
      }
      else if (strcmp(cmdbuf, "BLACKLIST CLEAR") == 0) {
        mesh_blacklist_clear();
        Serial.println("[MESH] Blacklist cleared — all relays can be used.\n");
      }
      else if (strcmp(cmdbuf, "MESH ON") == 0) {
        g_mesh_on = true;  g_addr_enabled = true;
        g_tx_dest_addr = 0xFF;
        Serial.println("[MESH] Relay protocol ENABLED (addr filter forced ON).");
        radio_init(cfg_rateIdx);
        Serial.println("[MESH] Done.\n");
      }
      else if (strcmp(cmdbuf, "MESH OFF") == 0) {
        g_mesh_on = false;
        Serial.println("[MESH] Relay protocol DISABLED.");
        Serial.printf("[MESH] Address filtering still %s.\n\n",
                       g_addr_enabled ? "ON" : "OFF");
      }
      else if (strcmp(cmdbuf, "ADDR OFF") == 0) {
        g_addr_enabled = false;
        g_addr_debug = false;
        Serial.println("[MESH] Address filtering DISABLED — radio accepts all packets.");
        radio_init(cfg_rateIdx);
        Serial.println("[MESH] Done.\n");
      }
      else if (strcmp(cmdbuf, "ADDR ON") == 0) {
        g_addr_enabled = true;
        Serial.println("[MESH] Address filtering ENABLED.");
        radio_init(cfg_rateIdx);
        Serial.println("[MESH] Done.\n");
      }
      else if (strcmp(cmdbuf, "ADDR DEBUG") == 0) {
        g_addr_debug = !g_addr_debug;
        Serial.printf("[MESH] FIFO debug logging %s\n\n", g_addr_debug ? "ON" : "OFF");
      }
      else if (strncmp(cmdbuf, "ADDR ", 5) == 0) {
        u8 a = (u8)strtoul(cmdbuf + 5, NULL, 16);
        if (a == 0x00 || a == 0xFF) {
          Serial.println("[MESH] Cannot use 0x00 or 0xFF — reserved for broadcast.\n");
        } else {
          g_node_addr = a;
          Serial.printf("[MESH] Node address set to 0x%02X (%u)\n", a, a);
          Serial.println("  Re-init radio to apply...");
          radio_init(cfg_rateIdx);
          Serial.println("[MESH] Done.\n");
        }
      }
      else if (strncmp(cmdbuf, "DEST ", 5) == 0) {
        u8 d = (u8)strtoul(cmdbuf + 5, NULL, 16);
        if (g_mesh_on) {
          g_mesh_final_dest = d;
          /* Radio dest will be computed from route table at TX time */
          Serial.printf("[MESH] Mesh dest set to 0x%02X%s\n",
                        d, (d == 0xFF || d == 0x00) ? " (BROADCAST)" : "");
          MeshRoute *r = mesh_route_find(d);
          if (r) Serial.printf("  Route: via 0x%02X (%u hops)\n", r->next_hop, r->hops);
          else   Serial.println("  No route yet — will broadcast until discovered");
          Serial.println();
        } else {
          g_tx_dest_addr = d;
          Serial.printf("[MESH] TX dest set to 0x%02X (%u)%s\n\n",
                        d, d, (d == 0xFF || d == 0x00) ? " (BROADCAST)" : "");
        }
      }
      else if (strcmp(cmdbuf, "PWR?") == 0) {
        Serial.printf("[PWR] TX Power = %s dBm (idx %u)\n\n", pwr_labels[cfg_pwrIdx], cfg_pwrIdx);
      }
      else if (strcmp(cmdbuf, "PWR LOW") == 0 || strcmp(cmdbuf, "PWR -10") == 0) {
        cfg_pwrIdx = 0;
        Serial.println("[PWR] TX Power → -10 dBm.  Re-init radio...");
        radio_init(cfg_rateIdx);
        Serial.println("[PWR] Done.\n");
      }
      else if (strcmp(cmdbuf, "PWR HIGH") == 0 || strcmp(cmdbuf, "PWR 20") == 0 || strcmp(cmdbuf, "PWR +20") == 0) {
        cfg_pwrIdx = 1;
        Serial.println("[PWR] TX Power → +20 dBm.  Re-init radio...");
        radio_init(cfg_rateIdx);
        Serial.println("[PWR] Done.\n");
      }
      else if (strcmp(cmdbuf, "HELP") == 0 || strcmp(cmdbuf, "?") == 0) {
        Serial.println("\n=== Serial Commands ===");
        Serial.println("  AR?         — dump latest test results");
        Serial.println("  AR? LIST    — list all saved tests (1-line each)");
        Serial.println("  AR? 1..10   — dump specific test from history (1=newest)");
        Serial.println("  AR? ALL     — dump all saved tests");
        Serial.println("  AR? CLEAR   — clear test results only");
        Serial.println("  EE WIPE     — wipe entire EEPROM (logs + tests)");
        Serial.println("  --- TX Power ---");
        Serial.println("  PWR?        — show current TX power");
        Serial.println("  PWR LOW     — set -10 dBm (short range test)");
        Serial.println("  PWR HIGH    — set +20 dBm (max range)");
        Serial.println("  --- Mesh Addressing ---");
        Serial.println("  MESH?       — show mesh config + route table");
        Serial.println("  MESH ON/OFF — enable/disable relay protocol");
        Serial.println("  ADDR xx     — set node address (hex, 01-FE)");
        Serial.println("  DEST xx     — set mesh dest (hex, FF=broadcast)");
        Serial.println("  ADDR ON/OFF — enable/disable HW address filter");
        Serial.println("  ADDR DEBUG  — toggle FIFO byte dump logging");
        Serial.println("  ROUTE?      — show route table + blacklist");
        Serial.println("  ROUTE CLEAR — clear routes + blacklist");
        Serial.println("  BLACKLIST?  — show dead-relay blacklist");
        Serial.println("  BLACKLIST CLEAR — force-clear blacklist");
        Serial.println("  HELP        — show this help");
        Serial.println("========================\n");
      }
      else {
        Serial.printf("[cmd] Unknown: '%s'\n", cmdbuf);
        Serial.println("Try: MESH?  ROUTE?  DEST xx  AR?  HELP\n");
      }
      cmdlen = 0;
    } else if (ch == 0x08 || ch == 0x7F) {  /* backspace */
      if (cmdlen > 0) cmdlen--;
    } else if (cmdlen < sizeof(cmdbuf) - 1) {
      cmdbuf[cmdlen++] = ch;
    }
  }
}

/* ─── OLED results display (scrollable, one config per screen) ─── */
static void ar_oled_results() {
  if (!_oledOK) return;
  uint8_t view = 0;
  bool redraw = true;

  while (true) {
    feedLoop();
    if (redraw) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tr);
      char buf[22];

      /* Line 1: Config header */
      snprintf(buf, sizeof(buf), "Cfg%u:%s %uB %s",
               view + 1, rate_labels[ar_rate[view]], size_opts[ar_size[view]],
               ar_my[view].valid ? "OK" : "FAIL");
      u8g2.setFont(u8g2_font_7x13B_tr);
      u8g2.drawStr(0, 12, buf);
      u8g2.setFont(u8g2_font_5x7_tr);

      ARRes *my = &ar_my[view];
      if (ar_is_master) {
        snprintf(buf, sizeof(buf), "Tx:%u Ok:%u Fl:%u AE:%u",
                 my->tx_sent, my->tx_ok, my->tx_fail, my->tx_corrupt);
        u8g2.drawStr(0, 22, buf);
        if (my->rtt_cnt > 0) {
          u32 ra = my->rtt_sum / my->rtt_cnt;
          snprintf(buf, sizeof(buf), "RTT:%lu/%lu/%lu",
                   (unsigned long)my->rtt_min, (unsigned long)my->rtt_max, (unsigned long)ra);
          u8g2.drawStr(0, 30, buf);
        }
      } else {
        snprintf(buf, sizeof(buf), "Rx:%u Dp:%u Er:%u",
                 my->rx_recv, my->rx_dup, my->rx_err);
        u8g2.drawStr(0, 22, buf);
      }

      if (my->rssi_cnt > 0) {
        int16_t avg = (int16_t)(my->rssi_sum / my->rssi_cnt);
        snprintf(buf, sizeof(buf), "RSSI:%d/%d/%d",
                 (int)my->rssi_min, (int)my->rssi_max, (int)avg);
        u8g2.drawStr(0, 38, buf);
      }

      float ber = my->total_bits > 0 ? (float)my->error_bits / (float)my->total_bits : 0;
      snprintf(buf, sizeof(buf), "BER:%.1e Rst:%u", ber, my->restarts);
      u8g2.drawStr(0, 46, buf);

      if (ar_got_peer && ar_peer[view].valid) {
        ARRes *pr = &ar_peer[view];
        int16_t pa = pr->rssi_cnt > 0 ? (int16_t)(pr->rssi_sum / pr->rssi_cnt) : 0;
        snprintf(buf, sizeof(buf), "Peer Rx:%u RSSI:%d", pr->rx_recv, (int)pa);
        u8g2.drawStr(0, 54, buf);
      }

      snprintf(buf, sizeof(buf), "%u/%u [U/D:nav OK:exit]", view + 1, ar_num_cfg);
      u8g2.drawStr(0, 63, buf);
      u8g2.sendBuffer();
      redraw = false;
    }

    feedLoop();
    Buttons b = readButtons();
    if (b.down && view < ar_num_cfg - 1) { view++; redraw = true; }
    if (b.up && view > 0) { view--; redraw = true; }
    if (b.ok) return;
    delay(5);
  }
}

/* ═══════════════════════════════════════════════════════════════
   ███ autoRangeRun — main orchestrator                       ███
   ═══════════════════════════════════════════════════════════════ */
static void autoRangeRun() {
  applyDIPConfig();   /* DIP2-6 → mesh addr config */
  /* ─── Role selection screen ──────────────────────────────────
     Default from DIP switch; user can toggle with UP/DOWN, confirm OK.
     LEFT exits back to menu.                                         */
  { Buttons b = _rawButtons();
    ar_is_master = !(b.dip & 0x01);   /* DIP[0]=0 → default master (A) */
  }
  delay(150);                          /* debounce after menu OK press */

  { /* Role selection — edge-detect, no blocking release-waits */
    uint8_t prev_btns = 0;
    u32 last_edge = millis();
    bool redraw = true;
    const u32 DEBOUNCE = 200;          /* ms between repeat presses */

    for (;;) {
      feedLoop();

      /* Redraw OLED only when selection changed */
      if (redraw && _oledOK) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x13B_tr);
        u8g2.drawStr(0, 13, "Auto Range");
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.drawStr(0, 29, "Select role:");
        if (ar_is_master) {
          u8g2.drawBox(0, 33, 128, 13);
          u8g2.setDrawColor(0);
          u8g2.drawStr(4, 44, "> MASTER (A)");
          u8g2.setDrawColor(1);
          u8g2.drawStr(4, 57, "  SLAVE  (B)");
        } else {
          u8g2.drawStr(4, 44, "  MASTER (A)");
          u8g2.drawBox(0, 46, 128, 13);
          u8g2.setDrawColor(0);
          u8g2.drawStr(4, 57, "> SLAVE  (B)");
          u8g2.setDrawColor(1);
        }
        u8g2.sendBuffer();
        Serial.printf("[AR] Role: %s — OK=confirm  UP/DN=change  LEFT=back\n",
                      ar_is_master ? "MASTER" : "SLAVE");
        redraw = false;
      }

      delay(20);
      uint8_t p1 = i2cRead(PCA_ADDR, 0x01);
      /* Pack current button levels: bit0=Up bit1=Down bit2=OK bit3=Left */
      uint8_t cur = 0;
      if (!(p1 & (1 << 5))) cur |= 0x01;  /* UP */
      if (!(p1 & (1 << 3))) cur |= 0x02;  /* DOWN */
      if (!(p1 & (1 << 6))) cur |= 0x04;  /* OK */
      if (!(p1 & (1 << 4))) cur |= 0x08;  /* LEFT */
      uint8_t edges = cur & ~prev_btns;    /* newly pressed */
      prev_btns = cur;

      if (!edges) continue;                /* nothing new */
      if (millis() - last_edge < DEBOUNCE) continue;
      last_edge = millis();

      if (edges & 0x04) break;             /* OK → confirmed */
      if (edges & 0x08) { appState = APP_STOPPED; return; }  /* LEFT → back */
      if (edges & 0x03) { ar_is_master = !ar_is_master; redraw = true; }
    }
  }

  if (ar_is_master) { my_id = 'A'; dest_id = 'B'; }
  else              { my_id = 'B'; dest_id = 'A'; }

  /* ─── Preset selection (master only) ─────────────────────────
     Slave doesn't need this — it learns configs from master's
     CONFIG packets as they arrive.                                */
  if (ar_is_master) {
    uint8_t sel = AR_PRESET_LARGE;  /* default */
    uint8_t prev_btns = 0;
    u32 last_edge = millis();
    bool redraw = true;
    const u32 DEBOUNCE = 200;

    for (;;) {
      feedLoop();
      if (redraw && _oledOK) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x13B_tr);
        u8g2.drawStr(0, 13, "Packet Sizes");
        u8g2.setFont(u8g2_font_6x10_tr);
        for (uint8_t i = 0; i < 3; i++) {
          int16_t y = 29 + i * 13;
          if (i == sel) {
            u8g2.drawBox(0, y - 10, 128, 13);
            u8g2.setDrawColor(0);
          }
          char lbl[22];
          snprintf(lbl, sizeof(lbl), "%c %s", (i == sel) ? '>' : ' ',
                   ar_preset_labels[i]);
          u8g2.drawStr(4, y, lbl);
          if (i == sel) u8g2.setDrawColor(1);
        }
        u8g2.sendBuffer();
        Serial.printf("[AR] Preset: %s — OK=confirm  UP/DN=change\n",
                      ar_preset_labels[sel]);
        redraw = false;
      }

      delay(20);
      uint8_t p1 = i2cRead(PCA_ADDR, 0x01);
      uint8_t cur = 0;
      if (!(p1 & (1 << 5))) cur |= 0x01;  /* UP */
      if (!(p1 & (1 << 3))) cur |= 0x02;  /* DOWN */
      if (!(p1 & (1 << 6))) cur |= 0x04;  /* OK */
      if (!(p1 & (1 << 4))) cur |= 0x08;  /* LEFT */
      uint8_t edges = cur & ~prev_btns;
      prev_btns = cur;
      if (!edges) continue;
      if (millis() - last_edge < DEBOUNCE) continue;
      last_edge = millis();

      if (edges & 0x04) break;             /* OK */
      if (edges & 0x08) { appState = APP_STOPPED; return; }
      if (edges & 0x01 && sel > 0) { sel--; redraw = true; }
      if (edges & 0x02 && sel < 2) { sel++; redraw = true; }
    }
    ar_setup_preset(sel);
    Serial.printf("[AR] Selected preset: %s (%u configs)\n",
                  ar_preset_labels[sel], ar_num_cfg);
  } else {
    /* Slave: start with 0 configs, will be built from CONFIG packets */
    ar_num_cfg = 0;
    memset(ar_rate, 0, sizeof(ar_rate));
    memset(ar_size, 0, sizeof(ar_size));
  }

  Serial.printf("\n[AR] Auto Range Test — Role: %s  ID: %c\n",
                ar_is_master ? "MASTER" : "SLAVE", my_id);

  /* Init results */
  for (uint8_t c = 0; c < AR_MAX_CFG; c++) { ar_res_init(&ar_my[c]); ar_res_init(&ar_peer[c]); }
  ar_got_peer = false;

  /* Phase 1: Discovery */
  if (!ar_discover()) {
    oledStatus("Auto Range", "Discovery", "FAILED!", "", "Press OK");
    appState = APP_STOPPED;
    return;
  }
  oledStatus("Auto Range", "Node Found!", "Starting test...");
  delay(1000);

  if (ar_is_master) {
    /* ─── MASTER: drive the test ─── */
    for (uint8_t ci = 0; ci < ar_num_cfg; ci++) {
      Serial.printf("\n[AR] ═══ Config %u/%u: %s × %uB ═══\n",
                    ci + 1, ar_num_cfg, rate_labels[ar_rate[ci]], size_opts[ar_size[ci]]);
      oledStatus("Auto Range", "Config switch...",
                 rate_labels[ar_rate[ci]]);

      if (!ar_switch_cfg(ci)) {
        Serial.printf("[AR] Cfg %u switch failed — skipping\n", ci);
        ar_my[ci].valid = false;
        continue;
      }

      if (!ar_master_test(ci)) {
        /* User cancelled */
        go_ready(); delay(2);
        Serial.println("[AR] Test cancelled by user");
        appState = APP_STOPPED;
        ar_serial_dump();
        ar_oled_results();
        return;
      }
    }

    /* Phase 3: Switch to safe config for result exchange, then collect.
       Result packet = 3 + ar_num_cfg*20 bytes. If current data_len is
       smaller than that, we need to switch both sides to 250B.            */
    oledStatus("Auto Range", "Getting peer", "results...");
    {
      uint16_t result_sz = 3 + (uint16_t)ar_num_cfg * 20;
      if (data_len < result_sz) {
        Serial.printf("[AR] Switching to 250B for %uB result packet\n", result_sz);
        /* Send CONFIG with ci=0xFE to signal "exchange mode" — slave switches
           radio but doesn't touch test state (0xFE >= AR_MAX_CFG).           */
        for (int retry = 0; retry < 5; retry++) {
          feedLoop();
          pkt[0] = dest_id;  pkt[1] = AR_CONFIG;
          pkt[2] = cfg_rateIdx;  pkt[3] = 4;   /* 250B */
          pkt[4] = 0;  pkt[5] = 0;  pkt[6] = 0xFE;
          if (!ar_tx_blk(pkt, 7)) { delay(300); continue; }
          RFResult r = ar_rx_blk(2000);
          if (r == RF_RX_DONE && g_rxd_len >= 4 && rxbuf[1] == AR_CFG_ACK) {
            ar_apply_cfg(cfg_rateIdx, 4);
            delay(200);
            break;
          }
          delay(500);
        }
      }
    }
    ar_get_peer_results();

  } else {
    /* ─── SLAVE: respond to master commands ─── */
    ar_slave_loop();
    /* Detect preset from received config table */
    if (ar_num_cfg == 8) ar_preset = AR_PRESET_ALL;
    else if (ar_num_cfg == 4) {
      bool has_small = false, has_large = false;
      for (uint8_t c = 0; c < 4; c++) {
        if (ar_size[c] <= 2) has_small = true;  /* 32 or 64 */
        if (ar_size[c] >= 3) has_large = true;  /* 128 or 250 */
      }
      ar_preset = has_small ? AR_PRESET_SMALL : AR_PRESET_LARGE;
    }
  }

  /* Phase 4: Save + Display results */
  Serial.println("\n[AR] Test complete!");
  go_ready(); delay(2);

  ar_ee_save();                         /* persist to EEPROM — recall with "AR?" serial cmd */
  ar_serial_dump();
  { char msg[22]; snprintf(msg, sizeof(msg), "Test #%u saved", ar_test_num);
    oledStatus("Auto Range", "COMPLETE!", msg, "AR?/AR? LIST/AR? ALL", "Press OK for results");
  }

  /* Wait for user to see message, then show OLED results */
  { u32 t0 = millis();
    while (millis() - t0 < 3000) { feedLoop(); delay(10); }
  }

  ar_oled_results();
  appState = APP_STOPPED;
}

/* ═══════════════════════════════════════════════════════════════
   ███ START OPERATION                                        ███
   ═══════════════════════════════════════════════════════════════ */
static void startOperation() {
  applyDIPConfig();   /* re-read DIP switches each test start */
  resetStats();
  data_len = size_opts[cfg_sizeIdx];

  /* Enable 2-byte length field for payloads that exceed 1-byte FIFO capacity */
  g_2byte_len = (data_len > 250);

  if (cfg_mode == MODE_TX || cfg_mode == MODE_PER || cfg_mode == MODE_CW) {
    my_id = 'A'; dest_id = 'B';
  } else if (cfg_mode == MODE_RX) {
    my_id = 'B'; dest_id = 'A';
  } else { /* TRX — use DIP switch to pick side */
    Buttons b = _rawButtons();   /* direct read — readButtons() throttle can return dip=0 */
    if (b.dip & 0x01) { my_id = 'B'; dest_id = 'A'; trx_is_tx_turn = false; }
    else              { my_id = 'A'; dest_id = 'B'; trx_is_tx_turn = true;  }
  }

  ack_timeout_ms = calcACKTimeout();
  tx_timeout_ms  = calcTXTimeout();
  Serial.printf("[config] Mode=%s Rate=%s Size=%u ACK_tmo=%lu TX_tmo=%lu\n",
                mode_labels[cfg_mode], rate_labels[cfg_rateIdx], data_len,
                (unsigned long)ack_timeout_ms, (unsigned long)tx_timeout_ms);

  /* Init radio for all packet modes */
  g_st = ST_IDLE;
  if (cfg_mode != MODE_REG_DUMP && cfg_mode != MODE_VIEW_LOGS &&
      cfg_mode != MODE_AUTO_RANGE) {
    radio_init(cfg_rateIdx);
  }

  printCSVHeader();
  throughput_start = millis();

  switch (cfg_mode) {
    case MODE_TX:
      oledStatus("TX Mode", "Sending...", rate_labels[cfg_rateIdx]);
      send_test_packet();
      break;
    case MODE_RX:
      oledStatus("RX Mode", "Listening...", rate_labels[cfg_rateIdx]);
      rx_last_pkt_ms = millis();
      rf_start_rx(rxbuf, MAX_PAYLOAD, INFINITE);
      break;
    case MODE_TRX:
      oledStatus("TRX Mode", trx_is_tx_turn ? "TX first" : "RX first");
      if (trx_is_tx_turn) send_test_packet();
      else rf_start_rx(rxbuf, MAX_PAYLOAD, ack_timeout_ms * 2);
      break;
    case MODE_PER:
      oledStatus("PER Test", "Starting 1000 pkts");
      per_target = PER_COUNT;
      send_test_packet();
      break;
    case MODE_CW:
      oledStatus("CW TX", "Carrier on...");
      /* First packet will be sent in cw_loop */
      break;
    case MODE_REG_DUMP:
      radio_init(cfg_rateIdx);
      drawRegDump();
      appState = APP_STOPPED;
      return;
    case MODE_VIEW_LOGS:
      viewLogs();
      appState = APP_MENU;
      return;
    case MODE_AUTO_RANGE:
      autoRangeRun();    /* blocking — handles everything internally */
      return;
    default: break;
  }

  appState = APP_RUNNING;
}

/* ═══════════════════════════════════════════════════════════════
   ███ SETUP & LOOP                                           ███
   ═══════════════════════════════════════════════════════════════ */

void setup() {
  boardInit();
  applyDIPConfig();   /* read DIP switches → set mesh addr/role/broadcast */
  Serial.println("\n═══════════════════════════════════════");
  Serial.println("  Zb-Rf Board V1.0");
  Serial.println("  RF Characterization + Mesh Firmware");
  Serial.println("  by lohith D");
  Serial.println("  5-slot sweep: 1.2/2.4 | 1.2/3.6 | 2.4/5 | 9.6 | 19.2");
  Serial.printf("  Mesh addr: 0x%02X  dest: 0x%02X  filter: %s  relay: %s\n",
                g_node_addr, g_tx_dest_addr,
                g_addr_enabled ? "ON" : "OFF",
                g_mesh_on ? "ON" : "OFF");
  Serial.println("═══════════════════════════════════════");

  ar_setup_preset(AR_PRESET_SMALL);  /* default: Quick 128B */

  drawSplash();
  ledGreen();
  for (int i = 0; i < 6; i++) { delay(200); kick(); }
  ledOff();

  appState = APP_MENU;
}

void loop() {
  feedLoop();                    /* also polls serial commands (AR?, HELP) — see feedLoop() */
  switch (appState) {
    case APP_MENU:
      runMenu();
      startOperation();
      break;

    case APP_RUNNING:
      switch (cfg_mode) {
        case MODE_TX:  tx_loop();     break;
        case MODE_RX:  rx_loop();     break;
        case MODE_TRX: trx_loop();    break;
        case MODE_PER: per_tx_loop(); break;
        case MODE_CW:  cw_loop();     break;
        default: appState = APP_MENU; break;
      }
      break;

    case APP_STOPPED:
      /* Save log when stopping */
      { static bool logged = false;
        if (!logged && (round_num > 0 || pktCount > 0)) {
          saveLog();
          logged = true;
        }
        Buttons b = readButtons();
        if (b.ok) {
          logged = false;
          ledOff(); delay(80);
          Serial.println("[app] Returning to menu");
          appState = APP_MENU;
        }
      }
      break;

    case APP_CONN_LOST:
      /* Save log on connection lost too */
      { static bool cl_logged = false;
        if (!cl_logged && (round_num > 0 || pktCount > 0)) {
          saveLog();
          cl_logged = true;
        }
        Buttons b = readButtons();
        if (b.ok) {
          cl_logged = false;
          ledOff(); delay(80);
          appState = APP_MENU;
        }
      }
      break;
  }
}
