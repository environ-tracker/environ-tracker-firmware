#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <zephyr/sys/util.h>
#include <zephyr/fs/fs.h>

// NOTE: Only here temp
#include <zephyr/bluetooth/uuid.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(file_search);

#include "file_helpers.h"



extern struct fs_mount_t *mp;

















int test_searching(uint16_t major, uint16_t minor)
{
    char file_name[CONFIG_FILE_SYSTEM_MAX_FILE_NAME];
    char network_name[CONFIG_FILE_SYSTEM_MAX_FILE_NAME];
    int ret;
    struct location location;
    struct bt_uuid_128 test_uuid;
    

    char *test = "7aaf1b67-f3c0-4a54-b314-58fff1960a40";

    ret = bt_uuid_from_str(test, &test_uuid.uuid);

    bt_uuid_to_str(&test_uuid.uuid, network_name,
             CONFIG_FILE_SYSTEM_MAX_FILE_NAME);

    ret = snprintf(file_name, CONFIG_FILE_SYSTEM_MAX_FILE_NAME, 
                "%s/beacons/%s.bin", mp->mnt_point, network_name);
    if (ret > CONFIG_FILE_SYSTEM_MAX_FILE_NAME) {
        LOG_WRN("Filename was truncated, size was: %d", ret);
        return ret;
    }

    ret = find_network_file(&test_uuid.uuid);
    if (ret == 0) {
        ret = find_beacon(file_name, major, minor, &location);
    }

    k_msleep(5);

    LOG_WRN("Testing is_supported_network(): %d", 
            is_supported_network(&test_uuid.uuid));

    k_msleep(5);

    LOG_WRN("Testing is_supported_network(): %d", 
            is_supported_network(&test_uuid.uuid));

    k_msleep(5);

    return ret;
}