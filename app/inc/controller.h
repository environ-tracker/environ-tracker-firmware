#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <zephyr/kernel.h>

#define SHORT_PRESS_EVENT   (1 << 0)
#define LONG_PRESS_EVENT    (1 << 4)

#define LEFT_BUTTON_SHORT_EVENT (1 << 0)
#define RIGHT_BUTTON_SHORT_EVENT (1 << 1)
#define SELECT_BUTTON_SHORT_EVENT (1 << 2)
#define BUTTON_SHORT_EVENTS_ALL (LEFT_BUTTON_SHORT_EVENT | \
                                 RIGHT_BUTTON_SHORT_EVENT | \
                                 SELECT_BUTTON_SHORT_EVENT)

extern struct k_event gpio_events;

#endif /* CONTROLLER_H */