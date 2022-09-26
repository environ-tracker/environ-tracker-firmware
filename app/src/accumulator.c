#include <zephyr.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <logging/log.h>

#include "environ.h"
#include "file/file_helpers.h"

LOG_MODULE_REGISTER(accumulator);

#define ACCUMULATOR_STACK_SIZE  2048
#define ACCUMULATOR_PRIORITY    4


#define MAX_PATH_LEN    255
#define PARTITION_NODE DT_NODELABEL(lfs)

FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);

struct fs_mount_t *mp = &FS_FSTAB_ENTRY(PARTITION_NODE);

#ifdef CONFIG_APP_FLASH_ERASE
static int littlefs_flash_erase(unsigned int id)
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

static int increase_infile_value(char *fname)
{
	uint32_t boot_count = 0;
	struct fs_file_t file;
	int rc, ret;

	fs_file_t_init(&file);
	rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
	if (rc < 0) {
		LOG_ERR("FAIL: open %s: %d", log_strdup(fname), rc);
		return rc;
	}

	rc = fs_read(&file, &boot_count, sizeof(boot_count));
	if (rc < 0) {
		LOG_ERR("FAIL: read %s: [rd:%d]", log_strdup(fname), rc);
		goto out;
	}
	LOG_INF("%s read count:%u (bytes: %d)\n", fname, boot_count, rc);

	rc = fs_seek(&file, 0, FS_SEEK_SET);
	if (rc < 0) {
		LOG_ERR("FAIL: seek %s: %d", log_strdup(fname), rc);
		goto out;
	}

	boot_count += 1;
	rc = fs_write(&file, &boot_count, sizeof(boot_count));
	if (rc < 0) {
		LOG_ERR("FAIL: write %s: %d", log_strdup(fname), rc);
		goto out;
	}

	LOG_INF("%s write new boot count %u: [wr:%d]\n", fname,
		   boot_count, rc);

 out:
	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("FAIL: close %s: %d", log_strdup(fname), ret);
		return ret;
	}

	return (rc < 0 ? rc : 0);
}


void accumulator_thread(void *a, void *b, void *c)
{
    struct environ_data data = {0};
    struct fs_statvfs sbuf;
    char fname[32];
    int err;

    LOG_INF("Accumulator started");

	#ifdef CONFIG_APP_FLASH_ERASE
	littlefs_flash_erase((uintptr_t)mp->storage_dev);
	#endif

    snprintf(fname, sizeof(fname), "%s/boot_count", mp->mnt_point);

    err = fs_statvfs(mp->mnt_point, &sbuf);
    if (err < 0) {
        LOG_ERR("statvfs returned %d", err);
        goto out;
    }

    LOG_DBG("%s: bsize = %lu ; frsize = %lu ; blocks = %lu ; bfree = %lu\n",
	        mp->mnt_point, sbuf.f_bsize, sbuf.f_frsize, 
            sbuf.f_blocks, sbuf.f_bfree);

    err = increase_infile_value(fname);
	if (err) {
		goto out;
	}

	#ifdef APP_CONFIG_BEACON_FILES
	err = initialise_files(mp);
	if (err) {
		goto out;
	}
	#endif /* APP_CONFIG_BEACON_FILES */

	LOG_ERR("Find beacon return %d", test_searching(0, 1));

    while (1) {

        err = k_msgq_get(&environ_data_msgq, &data, K_FOREVER);
        if (err == 0) {
            LOG_DBG("environ data received:\r\n\t"
                    "T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d; "
                    "L: %d; IR: %d; UV: %d.%06d", data.temp.val1, 
                    data.temp.val2, data.press.val1, data.press.val2, 
                    data.humidity.val1, data.humidity.val2, data.gas_res.val1, 
                    data.gas_res.val2, data.vis_light.val1, data.ir_light.val1,
                    data.uv_index.val1, data.uv_index.val2);
        } else {
            LOG_ERR("environ data receive error: %d", err);
        }
    }

out:
    err = fs_unmount(mp);
    LOG_INF("%s unmount: %d", mp->mnt_point, err);
}

K_THREAD_DEFINE(accumulator_id, ACCUMULATOR_STACK_SIZE, accumulator_thread, NULL, NULL, NULL, ACCUMULATOR_PRIORITY, 0, 0);