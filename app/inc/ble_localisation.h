#ifndef BLE_LOCALISATION_H
#define BLE_LOCALISATION_H

#include <zephyr.h>

extern const k_tid_t ble_localisation_id;

extern struct k_msgq ibeacon_request_msgq;

#endif /* BLE_LOCALISATION_H */