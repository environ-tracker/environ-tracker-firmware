#ifndef GNSS_H
#define GNSS_H

#include <zephyr/kernel.h>

/* Event to toggle printing the GNSS NMEA sentences to console */
#define GNSS_TOGGLE_DEBUG   (1 << 0)

#define GNSS_SLEEP  (1 << 1)

#define GNSS_WAKEUP (1 << 2)

#define GNSS_EVENTS GNSS_TOGGLE_DEBUG | GNSS_SLEEP | GNSS_WAKEUP

/* Controls behaviour of the GNSS module */
extern struct k_event gnss_control_events;

#endif /* GNSS_H */