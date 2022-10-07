#ifndef LOCATION_H
#define LOCATION_H

#include <zephyr/kernel.h>

enum location_source {
    LOCATION_BLE = 0,
    LOCATION_GNSS,
};

/**
 * @brief A location in 1e7 degrees
 */
struct location {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct location_wrapper {
    enum location_source source;
    struct location location;
};

extern struct k_msgq location_msgq;

#endif /* LOCATION_H */