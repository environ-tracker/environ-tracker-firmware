#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(file);

#include "file/file_common.h"
#include "ble/ble_network.h"


#ifdef CONFIG_APP_INIT_BEACON_FILES
#include <bluetooth/uuid.h>
/* NOTE: This is currently the only supported iBeacon network UUID */
static const struct bt_uuid_128 supported_beacon_network = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x7aaf1b67, 0xf3c0, 0x4a54, 0xb314, 0x58fff1960a40));


extern struct fs_mount_t *mp;
#endif /* CONFIG_APP_INIT_BEACON_FILES */


#ifdef CONFIG_APP_FLASH_ERASE
int storage_flash_erase(unsigned int id)
{
	const struct flash_area *pfa;
	int rc;

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		LOG_ERR("FAIL: unable to find flash area %u: %d\n",
			id, rc);
		return rc;
	}

	LOG_PRINTK("Area %u at 0x%x on %s for %u bytes\n",
		   id, (unsigned int)pfa->fa_off, pfa->fa_dev_name,
		   (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		LOG_ERR("Erasing flash area ... %d", rc);
	}

	flash_area_close(pfa);
	return rc;
}
#endif /* CONFIG_APP_FLASH_ERASE */

#ifdef CONFIG_APP_INIT_BEACON_FILES
int initialise_files(void)
{
    char file_name[FILE_NAME_LEN], dir_name[FILE_NAME_LEN];
    char uuid[BT_UUID_STR_LEN];

    struct ibeacon_file_info file_contents[] = {
        {
            .id = BEACON_ID_INIT(0, 0),
            .location = {
                .longitude = 0,
                .latitude = 0,
                .altitude = 0
            }
        },
        {
            .id = BEACON_ID_INIT(0, 1),
            .location = {
                .longitude = 10,
                .latitude = 0,
                .altitude = 0
            }
        },
        {
            .id = BEACON_ID_INIT(0, 2),
            .location = {
                .longitude = 0,
                .latitude = 10,
                .altitude = 0
            }
        },
        {
            .id = BEACON_ID_INIT(0, 3),
            .location = {
                .longitude = 10,
                .latitude = 10,
                .altitude = 0
            }
        },
        {
            .id = BEACON_ID_INIT(0, 4),
            .location = {
                .longitude = 20,
                .latitude = 5,
                .altitude = 0
            }
        }
    };

    int err, ret;

    LOG_INF("Initialising files");

    ret = absolute_file_name(dir_name, "beacons");
    if (ret) {
        LOG_ERR("initialise_files: Error forming beacon dir name. (%d)", ret);
        return ret;
    }

    err = fs_mkdir(dir_name);
    if (err != 0 && err != -EEXIST) {
        LOG_ERR("Failed to make directory: %s. Error: %d", dir_name, err);
        return err;
    }

    bt_uuid_to_str(&supported_beacon_network.uuid, uuid, sizeof(uuid));
    ret = snprintf(file_name, sizeof(file_name), "beacons/%s", uuid);
    if (ret > sizeof(file_name)) {
        LOG_WRN("Truncated filename, size would be: %d", ret);
        return ret;
    }

    ret = write_file(file_name, (uint8_t *)&file_contents, 
            sizeof(file_contents));
    if (ret) {
        LOG_ERR("initialise_files: Error initialising beacon file. (%d)", ret);
    }

    return ret;
}
#endif /* CONFIG_APP_INIT_BEACON_FILES */