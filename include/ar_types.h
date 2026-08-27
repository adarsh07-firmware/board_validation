#ifndef AR_TYPES_H
#define AR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════
   ███ AUTO-RANGING TEST TYPES                             ███
   ═══════════════════════════════════════════════════════════════ */

#define AR_MAX_CFG  8       /* Max configurations for auto-ranging test */

/* Auto-range result type — holds all stats for one test configuration */
typedef struct {
  bool      valid;         /* Result is valid */
  uint8_t   restarts;      /* Number of restarts for this config */
  
  /* TX Stats */
  uint16_t  tx_sent;       /* Total packets sent */
  uint16_t  tx_ok;         /* ACK'd packets */
  uint16_t  tx_fail;       /* NACK'd packets */
  uint16_t  tx_corrupt;    /* TX error count */
  
  /* RTT (Round Trip Time) Stats */
  uint32_t  rtt_sum;       /* Sum of all RTT measurements */
  uint32_t  rtt_min;       /* Minimum RTT (µs) */
  uint32_t  rtt_max;       /* Maximum RTT (µs) */
  uint16_t  rtt_cnt;       /* Count of valid RTT measurements */
  
  /* RX Stats */
  uint16_t  rx_recv;       /* Total packets received */
  uint16_t  rx_dup;        /* Duplicate packets */
  uint16_t  rx_err;        /* RX errors (clean = rx_recv - rx_err) */
  
  /* RSSI (Received Signal Strength) Stats */
  int16_t   rssi_min;      /* Minimum RSSI (dBm) */
  int16_t   rssi_max;      /* Maximum RSSI (dBm) */
  uint32_t  rssi_sum;      /* Sum of RSSI values for average */
  uint16_t  rssi_cnt;      /* Count of RSSI measurements */
  
  /* BER (Bit Error Rate) Stats */
  uint32_t  total_bits;    /* Total bits transmitted */
  uint32_t  error_bits;    /* Total bit errors detected */
} ARRes;

#endif /* AR_TYPES_H */
