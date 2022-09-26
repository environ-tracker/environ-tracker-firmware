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

#define BEACON_LINE_SIZE 11
#define MAX_CACHED_NETWORKS 4

extern struct fs_mount_t *mp;



int search_directory(struct fs_dir_t *dir, char *dir_name, char *file_name)
{
    int err = 0, ret = 0;

    if (strlen(file_name) > CONFIG_FILE_SYSTEM_MAX_FILE_NAME) {
        return -ENAMETOOLONG;
    }

    LOG_DBG("dir_name: %s, file_name: %s", dir_name, file_name);

    err = fs_opendir(dir, dir_name);
    if (err) {
        LOG_ERR("Error %d while opening dir: %s", err, log_strdup(dir_name));
        return err;
    }

    while (1) {
        // ret = 0;
        // break;
        struct fs_dirent entry;        

        err = fs_readdir(dir, &entry);
        if (err) {
            LOG_ERR("Error %d while reading dir: %s", err, 
                    log_strdup(dir_name));
            break;
        }

        if (entry.name[0] == '\0') {
            ret = -ENOENT;
            break;
        }

        // TODO: change to strncmp
        if (entry.type == FS_DIR_ENTRY_FILE && 
                (strcmp(entry.name, file_name) == 0)) {
            ret = 0;
            break;
        }
    }
    
    err = fs_closedir(dir);
    if (err) {
        LOG_ERR("Error %d closing dir: %s", err, log_strdup(dir_name));
    }

    return (err) ? err : ret;
}

int find_network_file(const struct bt_uuid *uuid)
{
    struct fs_dir_t dir;
    char network_name[CONFIG_FILE_SYSTEM_MAX_FILE_NAME];
    char file_name[CONFIG_FILE_SYSTEM_MAX_FILE_NAME];
    int ret;

    fs_dir_t_init(&dir);

    bt_uuid_to_str(uuid, network_name,
             CONFIG_FILE_SYSTEM_MAX_FILE_NAME);

    ret = snprintf(file_name, CONFIG_FILE_SYSTEM_MAX_FILE_NAME, "%s.bin", 
            network_name);
    if (ret > CONFIG_FILE_SYSTEM_MAX_FILE_NAME) {
        LOG_WRN("Filename was truncated, size was: %d", ret);
        return ret;
    }

    ret = snprintf(network_name, CONFIG_FILE_SYSTEM_MAX_FILE_NAME, "%s/beacons", 
            mp->mnt_point);
    if (ret > CONFIG_FILE_SYSTEM_MAX_FILE_NAME) {
        LOG_WRN("Filename was truncated, size was: %d", ret);
        return ret;
    }

    return search_directory(&dir, network_name, file_name);
    // return 0;
}


int bt_uuid_from_str(char *uuid, struct bt_uuid *out)
{
    uint8_t buf[BT_UUID_SIZE_128], temp[2];
    int j = 0, err;

    const int uuid_len = strlen(uuid);
    if (uuid_len % 2 != 0) {
        return -EINVAL;
    }

    for (int i = BT_UUID_SIZE_128 - 1; i >= 0; --i) {
        for (int k = 0; k < 2; ++k) {
            if (uuid[j + k] == '-') {
                j++;
            }

            err = char2hex(uuid[j + k], &temp[k]);
            if (err) {
                return err;
            }
        }
        buf[i] = temp[0] << 4 | temp[1];
        j += 2;
    }

    return bt_uuid_create(out, buf, BT_UUID_SIZE_128);
}


int find_beacon(char *fname, uint16_t major, uint16_t minor, 
        struct location *beacon_location)
{
    struct fs_file_t file;
    int rc, ret;
    uint8_t line[BEACON_LINE_SIZE + 1] = {0};

    fs_file_t_init(&file);

    rc = fs_open(&file, fname, FS_O_READ);
    if (rc == -EEXIST) {
        LOG_INF("File %s doesn't exist", log_strdup(fname));
        return rc;
    } else if (rc < 0) {
        LOG_ERR("Failed to open file %s, error %d", log_strdup(fname), rc);
        return rc;
    }
    
    while (1) {
        rc = fs_read(&file, line, BEACON_LINE_SIZE);
        if (rc < 0) {
            LOG_ERR("Failed to read line from %s, error: %d", 
                    log_strdup(fname), rc);
            break;
        } else if (rc != BEACON_LINE_SIZE) {
            LOG_WRN("Didn't read a whole line, read %d bytes", rc);
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
        LOG_ERR("Failed to close %s, error %d", log_strdup(fname), ret);
        return ret;
    }

    return (rc < 0 ? rc : 0);

}


int is_supported_network(const struct bt_uuid *network)
{
    static struct bt_uuid network_cache[MAX_CACHED_NETWORKS];
    static int cached_networks = 0;

    if (cached_networks) {
        for (int i = 0; i < cached_networks; ++i) {
            if (bt_uuid_cmp(&network_cache[i], network) == 0) {
                LOG_INF("Cache hit");
                return 0;
            }
        }
    }

    if (find_network_file(network) == 0) {
        if (cached_networks + 1 < MAX_CACHED_NETWORKS) {
            
            // TODO: remove magic 16 (aka bytes in 128bit)
            bt_uuid_create(&network_cache[cached_networks++], BT_UUID_128(network)->val, 16);
        }

        return 0;
    }

    return -1;
    // return 0;
}


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