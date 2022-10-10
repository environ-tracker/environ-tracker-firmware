#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/posix/time.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "ble/ble_network.h"
#include "file/file_common.h"

LOG_MODULE_REGISTER(ble_network);


#define MAX_CACHED_NETWORKS 4


struct network_cache_entry {
    struct bt_uuid entry;
    time_t last_access;
};

struct network_cache {
    int number_cached;
    struct network_cache_entry cache[MAX_CACHED_NETWORKS];
};

static struct network_cache network_cache;
K_MUTEX_DEFINE(network_cache_mutex);


static int network_is_cached(const struct bt_uuid *network)
{
    struct timespec time;
    int ret;

    ret = k_mutex_lock(&network_cache_mutex, K_MSEC(1));
    if (ret != 0) {
        return ret;
    }

    for (int i = 0; i < network_cache.number_cached; ++i) {
        if (bt_uuid_cmp(&network_cache.cache[i].entry, network) == 0) {
            clock_gettime(CLOCK_REALTIME, &time);
            
            network_cache.cache[i].last_access = time.tv_sec;
            ret = 0;
            goto out;
        }
    }

    ret = -EEXIST;

out: 
    k_mutex_unlock(&network_cache_mutex);
    return ret;
}

static int add_network_to_cache(const struct bt_uuid *network)
{
    int ret;

    ret = k_mutex_lock(&network_cache_mutex, K_MSEC(1));
    if (ret != 0) {
        return ret;
    }

    if (network_cache.number_cached < MAX_CACHED_NETWORKS) {
        bt_uuid_create(&network_cache.cache[network_cache.number_cached].entry, 
                BT_UUID_128(network)->val, BT_UUID_SIZE_128);

        network_cache.number_cached++;
    } else {
        int pos = 0;

        for (int i = 0; i < MAX_CACHED_NETWORKS; ++i) {
            if (network_cache.cache[i].last_access < 
                    network_cache.cache[pos].last_access) {
                pos = i;
            }
        }

        bt_uuid_create(&network_cache.cache[pos].entry, 
                BT_UUID_128(network)->val, BT_UUID_SIZE_128);
    }

    k_mutex_unlock(&network_cache_mutex);
    return ret;
}


bool is_supported_network(const struct bt_uuid *network)
{
    if (network_is_cached(network) == 0) {
        return true;
    }

    if (find_network(network) == 0) {
        add_network_to_cache(network);

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
    uint8_t buf[32];

    strcpy(network_file, "beacons/");
    position = strlen(network_file);
    
    bt_uuid_to_str(&beacon->network.uuid, &network_file[position], 
            FILE_NAME_LEN - position);

    ret = search_file(network_file, (uint8_t *)&beacon->id, sizeof(beacon->id), 
            0, buf, sizeof(struct ibeacon_file_info));
    if (ret != 0) {
        return ret;
    }

    if (beacon->id != sys_get_le32(buf)) {
        LOG_ERR("find_beacon: Incorrect beacon ID matched. Critical error in "
                "search_file function.");
        return -1;
    }

    uint8_t pos = sizeof(uint32_t);

    beacon->location.latitude = sys_get_le32(&buf[pos]);
    pos += sizeof(uint32_t);

    beacon->location.longitude = sys_get_le32(&buf[pos]);
    pos += sizeof(uint32_t);

    beacon->location.altitude = sys_get_le32(&buf[pos]);


    return 0;
}