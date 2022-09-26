#ifndef FILE_HELPERS_H
#define FILE_HELPERS_H

#include <zephyr/fs/fs.h>
#include <zephyr/bluetooth/uuid.h>

struct location {
    uint8_t x;
    uint8_t y;
    uint8_t z;
};


int search_directory(struct fs_dir_t *, char *, char *);

int find_network_file(const struct bt_uuid *);

int find_beacon(char *, uint16_t, uint16_t, struct location *);

int test_searching(uint16_t, uint16_t);

int bt_uuid_from_str(char *, struct bt_uuid *);

int is_supported_network(const struct bt_uuid *);

#endif /* FILE_HELPERS_H */