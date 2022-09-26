#include <stdio.h>
#include <fs/fs.h>
#include <zephyr/fs/littlefs.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(file, LOG_LEVEL_INF);

#include "file/file_common.h"


#define PARTITION_NODE DT_NODELABEL(lfs)

// NOTE: potentially needs a mutex
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);
struct fs_mount_t *mp = &FS_FSTAB_ENTRY(PARTITION_NODE);



int absolute_file_name(char *path, char *fname)
{
    int ret = snprintf(path, FILE_NAME_LEN, "%s/%s", mp->mnt_point, fname);
    if (ret > FILE_NAME_LEN) {
        LOG_ERR("construct_file_name: Attempted to write %d bytes", ret);
        return ret;
    }

    return 0;
}

int read_file(char *fname, uint8_t *data, uint32_t len)
{
    struct fs_file_t file;
    int rc, ret;

    fs_file_t_init(&file);
	rc = fs_open(&file, fname, FS_O_READ);
	if (rc < 0) {
		LOG_ERR("FAIL: open %s: %d", fname, rc);
		return rc;
	}

	rc = fs_read(&file, data, len);
	if (rc < 0) {
		LOG_ERR("FAIL: read %s: [rd:%d]", fname, rc);
		goto out;
	}

	LOG_INF("%s read %d bytes)\n", fname, rc);

out:
	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("FAIL: close %s: %d", fname, ret);
		return ret;
	}

	return (rc < 0 ? rc : 0);
}

int write_file(char *fname, uint8_t *data, uint32_t len)
{
    struct fs_file_t file;
    int rc, ret;

    fs_file_t_init(&file);
	rc = fs_open(&file, fname, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		LOG_ERR("FAIL: open %s: %d", fname, rc);
		return rc;
	}

    rc = fs_seek(&file, 0, FS_SEEK_SET);
    if (rc < 0) {
        LOG_ERR("FAIL: seek %s: %d", fname, rc);
        goto out;
    }

    rc = fs_write(&file, data, len);
	if (rc < 0) {
		LOG_ERR("FAIL: write %s: %d", fname, rc);
		goto out;
	}

    LOG_INF("%s wrote %d bytes", fname, rc);

    out:
	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("FAIL: close %s: %d", fname, ret);
		return ret;
	}

	return (rc < 0 ? rc : 0);
}