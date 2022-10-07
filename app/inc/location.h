#ifndef LOCATION_H
#define LOCATION_H

#include <zephyr/kernel.h>

/**
 * @brief A location in 1e7 degrees
 */
struct location {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

extern struct k_msgq location_msgq;

#endif /* LOCATION_H */