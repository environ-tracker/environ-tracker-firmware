#ifndef BLE_NETWORK_H
#define BLE_NETWORK_H

#include <bluetooth/uuid.h>


#define BEACON_LINE_SIZE 11
#define MAX_CACHED_NETWORKS 4


struct location {
    uint8_t x;
    uint8_t y;
    uint8_t z;
};

/**
 * @brief Checks to see if the given network is supported by the localisation 
 *        engine.
 * 
 * @param network Network UUID to check 
 * @return true if supported
 */
bool is_supported_network(const struct bt_uuid *network);

/**
 * @brief Finds if the given network's beacon list file exists.
 * 
 * @param network Network UUID to search
 * @return 0 on success, else negative error code
 */
int find_network(const struct bt_uuid *network);

int find_beacon(char * fname, uint16_t major, uint16_t minor, struct location *location);

#endif /* BLE_NETWORK_H */