#ifndef BLE_NETWORK_H
#define BLE_NETWORK_H

#include <bluetooth/uuid.h>
#include <kernel.h>

#include "location.h"


#define BEACON_LINE_SIZE 11
#define MAX_CACHED_NETWORKS 4

#define BEACON_ID_INIT(major, minor) (major << 16) | minor


/**
 * @brief The core details to store about a beacon
 */
struct ibeacon {
    struct bt_uuid_128 network_uuid;
    uint32_t id;
    struct location location;
} __packed;

/**
 * @brief Information about the beacon stored in a file
 */
struct ibeacon_file_info {
    uint32_t id;
    struct location location;
} __packed;

/**
 * @brief Data received from an iBeacon packet
 */
struct ibeacon_packet {
    struct ibeacon beacon;
    int64_t discovered_time;
    int8_t tx_power;
    int8_t rssi;
    sys_snode_t next;
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

/**
 * @brief Retrieve a beacons location based off its network and beacon IDs.
 * 
 * @param beacon Beacon to retrieve location of
 * @return 0 on succes, else negative error code
 */
int find_beacon(struct ibeacon *beacon);

#endif /* BLE_NETWORK_H */