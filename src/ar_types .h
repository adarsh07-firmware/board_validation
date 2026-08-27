/* ar_types.h — Auto-range test types.
 * Placed in a header so the Arduino IDE's auto-generated function
 * prototypes can resolve the ARRes type.  The IDE inserts prototypes
 * right after the last #include; by putting the typedef *inside* an
 * include it is available before those prototypes.                     */
#ifndef AR_TYPES_H
#define AR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#define AR_MAX_CFG   8           /* max configs (8 = Full preset: 5 rates × 128B + 3 rates × 250B) */

typedef struct {
  bool     valid;  uint8_t  restarts;
  uint16_t tx_sent, tx_ok, tx_fail, tx_corrupt;
  uint32_t rtt_sum, rtt_min, rtt_max;  uint16_t rtt_cnt;
  uint16_t rx_recv, rx_dup, rx_err;
  int16_t  rssi_min, rssi_max;  int32_t rssi_sum;  uint16_t rssi_cnt;
  uint32_t total_bits, error_bits;
} ARRes;

#endif /* AR_TYPES_H */
