#ifndef FILE_HELPERS_H
#define FILE_HELPERS_H

#include <zephyr/fs/fs.h>
#include <zephyr/bluetooth/uuid.h>

struct location {
    uint8_t x;
    uint8_t y;
    uint8_t z;
};


int initialise_files(struct fs_mount_t *);

int search_directory(struct fs_dir_t *, char *, char *);

int find_network_file(char *);

int find_beacon(char *, uint16_t, uint16_t, struct location *);

int test_searching(uint16_t, uint16_t);

#endif /* FILE_HELPERS_H */