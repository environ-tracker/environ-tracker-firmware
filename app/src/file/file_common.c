#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file, LOG_LEVEL_INF);

#include "file/file_common.h"


#define PARTITION_NODE DT_NODELABEL(lfs)

// NOTE: potentially needs a mutex
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);
struct fs_mount_t *mp = &FS_FSTAB_ENTRY(PARTITION_NODE);

K_MUTEX_DEFINE(fs_mutex);


int absolute_file_name(char *path, char *fname)
{
    int ret = snprintf(path, FILE_NAME_LEN, "%s/%s", mp->mnt_point, fname);
    if (ret > FILE_NAME_LEN) {
        LOG_ERR("construct_file_name: Attempted to write %d bytes", ret);
        return ret;
    }

    return 0;
}

int delete_file(char *fname)
{
    char abs_fname[FILE_NAME_LEN];
    int rc;

    rc = absolute_file_name(abs_fname, fname);
    if (rc) {
        return rc;
    }

    rc = fs_unlink(abs_fname);
    if (rc) {
        LOG_ERR("delete_file: failed to delete %s. (%d)", fname, rc);    
    }

    return rc;
}

int read_file(char *fname, uint8_t *data, uint32_t len)
{
    struct fs_file_t file;
    char abs_fname[FILE_NAME_LEN];
    int rc, ret;

    rc = absolute_file_name(abs_fname, fname);
    if (rc) {
        return rc;
    }

    if ((rc = k_mutex_lock(&fs_mutex, K_FOREVER))) {
        return rc;
    }

    fs_file_t_init(&file);
    rc = fs_open(&file, abs_fname, FS_O_READ);
    if (rc < 0) {
        LOG_ERR("FAIL: open %s: %d", fname, rc);
        goto out2;
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
        k_mutex_unlock(&fs_mutex);
        return ret;
    }

out2:
    k_mutex_unlock(&fs_mutex);

    return (rc < 0 ? rc : 0);
}

int write_file(char *fname, uint8_t *data, uint32_t len)
{
    struct fs_file_t file;
    char abs_fname[FILE_NAME_LEN];
    int rc, ret;

    rc = absolute_file_name(abs_fname, fname);
    if (rc) {
        return rc;
    }

    if ((rc = k_mutex_lock(&fs_mutex, K_FOREVER))) {
        return rc;
    }

    fs_file_t_init(&file);
    rc = fs_open(&file, abs_fname, FS_O_CREATE | FS_O_WRITE);
    if (rc < 0) {
        LOG_ERR("FAIL: open %s: %d", fname, rc);
        goto out2;
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
        k_mutex_unlock(&fs_mutex);
        return ret;
    }
out2:
    k_mutex_unlock(&fs_mutex);

    return (rc < 0 ? rc : 0);
}

int search_directory(char *dir_name, char *file_name)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    char dir_path[FILE_NAME_LEN];
    int err = 0, ret = 0, rc = 0;

    // TODO: Use strnlen
    if (strlen(file_name) > FILE_NAME_LEN) {
        return -ENAMETOOLONG;
    }

    err = absolute_file_name(dir_path, dir_name);
    if (err) {
        return err;
    }

    LOG_DBG("dir_name: %s, file_name: %s", dir_name, file_name);

    if ((rc = k_mutex_lock(&fs_mutex, K_FOREVER))) {
        return rc;
    }

    fs_dir_t_init(&dir);
    err = fs_opendir(&dir, dir_path);
    if (err) {
        LOG_ERR("Error %d while opening dir: %s", err, dir_name);
        goto out;
    }

    while (1) {
        err = fs_readdir(&dir, &entry);
        if (err) {
            LOG_ERR("Error %d while reading dir: %s", err, dir_name);
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
    
    err = fs_closedir(&dir);
    if (err) {
        LOG_ERR("Error %d closing dir: %s", err, dir_name);
    }

out:
    k_mutex_unlock(&fs_mutex);

    return (err) ? err : ret;
}

int search_file(char *fname, uint8_t *data, uint32_t len, uint32_t offset,
        uint8_t *block, uint32_t block_len)
{
    struct fs_file_t file;
    uint8_t line[block_len];
    char abs_fname[FILE_NAME_LEN];
    int rc, ret;

    rc = absolute_file_name(abs_fname, fname);
    if (rc) {
        return rc;
    }

    if ((rc = k_mutex_lock(&fs_mutex, K_FOREVER))) {
        return rc;
    }

    fs_file_t_init(&file);
    rc = fs_open(&file, abs_fname, FS_O_READ);
    if (rc == -EEXIST) {
        LOG_WRN("search_file: File %s doesn't exist", fname);
        goto out;
    } else if (rc < 0) {
        LOG_ERR("search_file: Failed opening %s. (%d)", fname, rc);
        goto out;
    }

    while (1) {
        rc = fs_read(&file, line, block_len);
        if (rc < 0) {
            LOG_ERR("search_file: Failed reading line from %s. (%d)", 
                    fname, rc);
            break;
        } else if (rc != block_len) {
            LOG_INF("search_file: Partial read of %d bytes from %s", rc, fname);
            rc = -ESRCH;
            break;
        }

        if (memcmp(line, data, len) == 0) {
            // data was found in the file
            LOG_HEXDUMP_INF(data, len, "was found in file");
            memcpy(block, line, block_len);
            break;
        }
    }

    ret = fs_close(&file);
    if (ret < 0) {
        LOG_ERR("search_file: Failed to close %s. (%d)", fname, ret);
        k_mutex_unlock(&fs_mutex);
        return ret;
    }

out:
    k_mutex_unlock(&fs_mutex);

    return (rc < 0 ? rc : 0);
}