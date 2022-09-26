#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(file);

#include "file/file_common.h"


/* NOTE: This is currently the only supported iBeacon network UUID */
static const struct bt_uuid_128 supported_beacon_network = BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x7aaf1b67, 0xf3c0, 0x4a54, 0xb314, 0x58fff1960a40));

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

#ifdef APP_CONFIG_BEACON_FILES
int initialise_files(struct fs_mount_t *mp)
{
    struct fs_file_t file;
    char fname[64], fname2[64];
    char uuid[50];
    char buffer[200];

    // NOTE: I think the endianess of these is wrong
    /* This is 4 beacons in the form: majorminor,x,y,z */
    uint8_t file_contents[] = {0, 0, 0, 0, ',', 0, ',', 0, ',', 0, '\n',
                               0, 0, 0, 1, ',', 10, ',', 0, ',', 0, '\n',
                               0, 0, 0, 3, ',', 0, ',', 10, ',', 0, '\n',
                               0, 0, 0, 4, ',', 10, ',', 10, ',', 0, '\n'};
    int err, ret;

    LOG_INF("Initialising files");

    ret = snprintf(fname2, sizeof(fname2), "%s/beacons", mp->mnt_point);
    if (ret > sizeof(fname2)) {
        LOG_WRN("Truncated filename, size would be: %d", ret);
        return ret;
    }

    err = fs_mkdir(fname2);
    if (err != 0 && err != -EEXIST) {
        LOG_ERR("Failed to make directory: %s. Error: %d", log_strdup(fname2), 
                err);
        return err;
    }

    bt_uuid_to_str(&supported_beacon_network.uuid, uuid, 50);
    ret = snprintf(fname, sizeof(fname), "%s/%s.bin", fname2, uuid);
    if (ret > sizeof(fname)) {
        LOG_WRN("Truncated filename, size would be: %d", ret);
        return ret; // TODO: change to a better errno
    }

    fs_file_t_init(&file);
    err = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
    if (err < 0) {
        LOG_ERR("Failed to open file %s. Error: %d", log_strdup(fname), err);
        return err;
    }

    err = fs_write(&file, file_contents, ARRAY_SIZE(file_contents));
    if (err < 0) {
        LOG_ERR("Failed to write to file %s. Error: %d", log_strdup(fname), err);
        goto out;
    }

    if (err != ARRAY_SIZE(file_contents)) {
        LOG_ERR("Didn't write entire array. Wrote %d bytes, array is %d bytes", err, ARRAY_SIZE(file_contents));
        goto out;
    }

    err = fs_sync(&file);
    if (err < 0) {
        LOG_ERR("Couldn't flush file. Error %d", err);
        goto out;
    }

    err = fs_seek(&file, 0, FS_SEEK_SET);
    if (err < 0) {
        LOG_ERR("Failed to seek to start of file, Error %d", err);
        goto out;
    }

    err = fs_read(&file, buffer, 200);
    if (err < 0) {
        LOG_ERR("Failed while reading. Error %d", err);
        goto out;
    }

    int offset;
    for (int i = 0; i < 4; ++i) {
        offset = i * 11;
        LOG_PRINTK("Major: %x%x, Minor: %x%x, x: %x, y: %x, z: %x\n", 
                buffer[offset + 0], buffer[offset + 1], buffer[offset + 2], 
                buffer[offset + 3], buffer[offset + 5], buffer[offset + 7], 
                buffer[offset + 9]);
    }

out:
    err = fs_close(&file);
    if (err < 0) {
        LOG_ERR("Failed to close file. Error %d", err);
        return err;
    }

    return (err < 0 ? err : 0);
}
#endif /* APP_CONFIG_BEACON_FILES */