#include <stdio.h>
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
    char network_name[FILE_NAME_LEN], file_name[FILE_NAME_LEN]
    int ret;

    bt_uuid_to_str(uuid, network_name, FILE_NAME_LEN);

    ret = snprintf(file_name, FILE_NAME_LEN, "%s.bin", network_name);
    if (ret >= FILE_NAME_LEN) {
        LOG_ERR("find_network: File name was truncated, size was: %d", ret);
        return ret;
    }

    ret = absolute_file_name(network_name, "beacons");
    if (ret != 0) {
        return ret;
    }

    return search_directory(network_name, file_name);
}


// TODO refactor this function. Clean up function params and code inside
int find_beacon(char *fname, uint16_t major, uint16_t minor, 
        struct location *beacon_location)
{
    struct fs_file_t file;
    uint8_t line[BEACON_LINE_SIZE + 1] = {0};
    int rc, ret;

    fs_file_t_init(&file);
    rc = fs_open(&file, fname, FS_O_READ);
    if (rc == -EEXIST) {
        LOG_INF("find_beacon: File %s doesn't exist", fname);
        return rc;
    } else if (rc < 0) {
        LOG_ERR("find_beacon: Failed opening %s. (%d)", fname, rc);
        return rc;
    }
    
    while (1) {
        rc = fs_read(&file, line, BEACON_LINE_SIZE);
        if (rc < 0) {
            LOG_ERR("find_beacon: Failed reading line from %s. (%d)", 
                    fname, rc);
            break;
        } else if (rc != BEACON_LINE_SIZE) {
            LOG_WRN("find_beacon: Partial read of %d bytes", rc);
            rc = -ESRCH;
            break;
        }

        // TODO: Fix endianess of file
        if (((uint16_t *)line)[0] == major && 
                (line[3] | line[2] << 8) == minor) {
            LOG_WRN("Found beacon, major: %d, minor: %d, x: %d, y: %d, z: %d", 
                    ((uint16_t *)line)[0], line[3] | line[2] << 8, 
                    line[5], line[7], line[9]);
            break;
        }
    }

    ret = fs_close(&file);
    if (ret < 0) {
        LOG_ERR("find_beacon: Failed to close %s. (%d)", fname, ret);
        return ret;
    }

    return (rc < 0 ? rc : 0);
}