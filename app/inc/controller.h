#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <zephyr/kernel.h>

#define SHORT_PRESS_EVENT   (1 << 0)
#define LONG_PRESS_EVENT    (1 << 4)

#define LEFT_BUTTON_SHORT_EVENT     (1 << 0)
#define RIGHT_BUTTON_SHORT_EVENT    (1 << 1)
#define SELECT_BUTTON_SHORT_EVENT   (1 << 2)
#define BUTTON_SHORT_EVENTS_ALL     (LEFT_BUTTON_SHORT_EVENT | \
                                     RIGHT_BUTTON_SHORT_EVENT | \
                                     SELECT_BUTTON_SHORT_EVENT)

#define LEFT_BUTTON_LONG_EVENT      (LONG_PRESS_EVENT << 0)
#define RIGHT_BUTTON_LONG_EVENT     (LONG_PRESS_EVENT << 1)
#define SELECT_BUTTON_LONG_EVENT    (LONG_PRESS_EVENT << 2)
#define BUTTON_LONG_EVENTS_ALL      (LEFT_BUTTON_LONG_EVENT | \
                                     RIGHT_BUTTON_LONG_EVENT | \
                                     SELECT_BUTTON_LONG_EVENT)

#define SOFT_SHUTDOWN_EVENT         (LEFT_BUTTON_LONG_EVENT | \
                                     RIGHT_BUTTON_LONG_EVENT)

#define BAT_CHARGING_STATE_CHANGE_EVENT  (1 << 0)

extern struct k_event gpio_events;

extern struct k_event power_events;

#endif /* CONTROLLER_H */