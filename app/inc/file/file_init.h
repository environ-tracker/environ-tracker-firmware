#ifndef FILE_INIT_H
#define FILE_INIT_H

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>

#ifdef CONFIG_APP_FLASH_ERASE
int storage_flash_erase(unsigned int id);
#endif

#ifdef APP_CONFIG_BEACON_FILES
int initialise_files(struct fs_mount_t *mp);
#endif /* APP_CONFIG_BEACON_FILES */

#endif /* FILE_INIT_H */