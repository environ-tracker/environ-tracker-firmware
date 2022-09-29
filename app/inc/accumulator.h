#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H

#include <zephyr.h>

struct system_data {
    uint32_t placeholder;
}__attribute__((aligned(4)));

extern struct k_msgq lorawan_msgq;

#endif /* ACCUMULATOR_H */