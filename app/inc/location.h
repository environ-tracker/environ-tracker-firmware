#ifndef LOCATION_H
#define LOCATION_H

#include <zephyr/kernel.h>

enum location_source {
    LOCATION_BLE = 0,
    LOCATION_GNSS,
};

/**
 * @brief A location
 */
struct location {
    float latitude;
    float longitude;
    float altitude;
};

struct location_wrapper {
    enum location_source source;
    struct location location;
};

extern struct k_msgq location_msgq;

/**
 * @brief Converts the given measurement from a float into an int32 given in 
 *        1e6 degrees.
 * 
 * @param measurement Measurement to convert
 * @return measurement in units of 1e6 degrees
 */
inline int32_t location_float_to_int32(float measurement) {
    return (int32_t)(measurement * 10 * 6);
}

#endif /* LOCATION_H */