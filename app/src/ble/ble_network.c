#include <stdio.h>
#include <string.h>
#include <bluetooth/uuid.h>
#include <fs/fs.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_network);

#include "ble/ble_network.h"
#include "file/file_common.h"


bool is_supported_network(const struct bt_uuid *network)
{
    // NOTE: Should cache be kept here??
    static struct bt_uuid network_cache[MAX_CACHED_NETWORKS];
    static int cached_networks = 0;

    if (cached_networks) {
        for (int i = 0; i < cached_networks; ++i) {
            if (bt_uuid_cmp(&network_cache[i], network) == 0) {
                LOG_INF("Cache hit");
                return true;
            }
        }
    }

    if (find_network(network) == 0) {
        if (cached_networks + 1 < MAX_CACHED_NETWORKS) {
            bt_uuid_create(&network_cache[cached_networks++], 
                    BT_UUID_128(network)->val, BT_UUID_SIZE_128);
        }

        return true;
    }

    return false;
}

int find_network(const struct bt_uuid *uuid)
{
    char network_name[FILE_NAME_LEN];

    bt_uuid_to_str(uuid, network_name, FILE_NAME_LEN);

    return search_directory("beacons", network_name);
}

int find_beacon(struct ibeacon *beacon)
{
    char network_file[FILE_NAME_LEN];
    int ret, position;

    strcpy(network_file, "beacons/");
    position = strlen(network_file);
    
    bt_uuid_to_str(&beacon->network_uuid.uuid, &network_file[position], 
            FILE_NAME_LEN - position);

    ret = search_file(network_file, (uint8_t *)&beacon->id, sizeof(beacon->id), 
            0, (uint8_t *)&beacon->id, 16);
    if (ret != 0) {
        return ret;
    }

    return 0;
}