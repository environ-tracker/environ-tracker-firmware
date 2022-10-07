#ifndef GNSS_H
#define GNSS_H

#include <zephyr.h>

/* Event to toggle printing the GNSS NMEA sentences to console */
#define GNSS_TOGGLE_DEBUG_OUTPUT   (1 << 0)

/* Controls behaviour of the GNSS module */
extern struct k_event gnss_control_events;

#endif /* GNSS_H */