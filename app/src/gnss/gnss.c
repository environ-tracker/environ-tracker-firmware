#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/posix/time.h>
#include <zephyr/drivers/rtc/rtc_rv3028.h>
#include <zephyr/logging/log.h>

#include "gnss/gnss.h"
#include "gnss/minmea.h"
#include "gnss/ubx/u_ubx_protocol.h"
#include "accumulator.h"
#include "location.h"


LOG_MODULE_REGISTER(gnss, CONFIG_LOG_DEFAULT_LEVEL);


/* Defines for the GNSS thread stack size and priority */
#define GNSS_STACK_SIZE 1024
#define GNSS_PRIORITY   5

#define GNSS_TIMEOUT 	5 * 60 * 1000
#define GNSS_SLEEP_TIME	K_MINUTES(5)


/* Local function prototypes */
static int32_t set_gnss_low_power_poll(void);
static int32_t set_gnss_nmea_zda_enable(void);
static int32_t sleep_gnss(uint32_t duration);
static inline void wakeup_gnss(void);
void serial_cb(const struct device *dev, void *user_data);
void gnss_thread(void *a, void *b, void *c);


/* Queue to store GNSS receiver messages */
K_MSGQ_DEFINE(uart_msgq, MINMEA_MAX_SENTENCE_LENGTH, 10, 4);

/* UART device for ZOE-M8Q GPS */
static const struct device *zoe_uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

/* Receive buffer used in UART ISR callback */
static char rx_buf[MINMEA_MAX_SENTENCE_LENGTH];
static int rx_buf_pos;

/* Controls behaviour of the GNSS module */
K_EVENT_DEFINE(gnss_control_events);

/* Initialise GNSS handler thread  */
K_THREAD_DEFINE(gnss_id, GNSS_STACK_SIZE, gnss_thread, NULL, NULL, NULL, 
        GNSS_PRIORITY, 0, 0);


/**
 * @brief Thread to handle GNSS interaction. 
 */
void gnss_thread(void *a, void *b, void *c)
{
    struct location_wrapper location = {0};
    char tx_buf[MINMEA_MAX_SENTENCE_LENGTH];
    bool debug_output_enable = false;
    uint32_t events;
    int64_t last_gps_fix_time;
    int rc;

    const struct device *rtc = DEVICE_DT_GET_ONE(microcrystal_rv3028);

    LOG_INF("GNSS thread started");

    if (!device_is_ready(rtc)) {
        LOG_ERR("%s: Device not ready.", rtc->name);
        return;
    }

    /* Check if GNSS receiver UART is enabled */
    if (!device_is_ready(zoe_uart_dev)) {
        LOG_ERR("%s: GNSS UART device not ready.", zoe_uart_dev->name);
        return;
    }

    /* Double check GNSS receiver is woken */ 
    wakeup_gnss();

    /* Set the GNSS into 1Hz super-efficient power mode */
    set_gnss_low_power_poll();

    /* Enable NMEA ZDA */
    set_gnss_nmea_zda_enable();

    /* Configure interrupt and callback to receive data */
    uart_irq_callback_user_data_set(zoe_uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(zoe_uart_dev);


    location.source = LOCATION_GNSS;

    last_gps_fix_time = k_uptime_get();

    LOG_INF("ZOE-M8Q UART setup");

    while (1) {
        /* Check what events need to be handled */
        events = k_event_wait(&gnss_control_events, GNSS_EVENTS, false, 
                K_NO_WAIT);
        if (events & GNSS_TOGGLE_DEBUG) {

            debug_output_enable = !debug_output_enable;
            k_event_set_masked(&gnss_control_events, 0, GNSS_TOGGLE_DEBUG);
        } else if (events & GNSS_SLEEP) {

            sleep_gnss(0);
            k_event_set_masked(&gnss_control_events, 0, GNSS_SLEEP);
        } else if (events & GNSS_WAKEUP) {

            wakeup_gnss();
            k_event_set_masked(&gnss_control_events, 0, GNSS_WAKEUP);
        }

        while (k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(50)) == 0) {
            if (debug_output_enable) {
                printk("%s", tx_buf);
            }

            /* Parse the received NMEA sentence */
            switch (minmea_sentence_id(tx_buf, false)) {
            case MINMEA_SENTENCE_GGA:
                struct minmea_sentence_gga frame;

                if (!minmea_parse_gga(&frame, tx_buf)) {
                    LOG_WRN("Issue parsing NMEA GGA message");
                    break;	
                } else if (frame.fix_quality == 0) {
                    /* No fix, ignore message */

                    if (k_uptime_get() - last_gps_fix_time > GNSS_TIMEOUT) {
                        LOG_INF("GNSS timed out with no fix, sleeping...");
                        
                        sleep_gnss(0);
                        k_sleep(GNSS_SLEEP_TIME);
                        wakeup_gnss();

                        last_gps_fix_time = k_uptime_get();
                        LOG_INF("GNSS woken");
                    }

                    break;
                }

                last_gps_fix_time = k_uptime_get();

                /* Store the location data in 1e7 degrees */
                location.location.latitude = minmea_tocoord(&frame.latitude);
                location.location.longitude = minmea_tocoord(&frame.longitude);

                /* Store the altitude in 1e4 meters */
                location.location.altitude = minmea_tofloat(&frame.altitude);
                LOG_INF("lat: %f, long: %f, alt: %f", location.location.latitude, location.location.longitude, location.location.altitude);

                /* Place the retrieved location data on the queue */
                while (k_msgq_put(&location_msgq, &location, K_NO_WAIT) != 0) {
                    k_msgq_purge(&location_msgq);
                    LOG_DBG("location_msgq has been purged");
                }

                k_event_post(&data_events, LOCATION_DATA_PENDING);

                break;
            case MINMEA_SENTENCE_ZDA:
                struct minmea_sentence_zda zda_frame;
                struct timespec ts;

                if (!minmea_parse_zda(&zda_frame, tx_buf)) {
                    LOG_WRN("Issue parsing NMEA ZDA message");
                    break;
                }

                rc = minmea_gettime(&ts, &zda_frame.date, &zda_frame.time);
                if (rc == -1) {
                    /* Invalid timespec */
                    continue;
                }

                clock_settime(CLOCK_REALTIME, &ts);
                rv3028_rtc_set_time(rtc, ts.tv_sec);

                LOG_INF("Clock set from GPS ZDA message");
            default:
                /* We don't care about these messages */
            }
        }
    }
}


/**
 * @brief Set the GNSS receiver into 1HZ super-efficient power mode.
 * 
 * @return 0 on success, else a negative error code from u_error_common.h
 */
static int32_t set_gnss_low_power_poll(void)
{
    char message[8] = {0};
    char encodedMessage[U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES + 8];
    int32_t ret;

    /* Sets 1Hz SuperE mode */
    message[1] = 0x03;

    /* Encode the UBX-CFG-PMS message. */
    ret = uUbxProtocolEncode(0x06, 0x86, message, sizeof(message), 
            encodedMessage);
    if (ret < 0) {
        LOG_ERR("set_gnss_low_power_poll: Error encoding packet. (%d)", ret);
        return ret;
    }

    /* Send UBX message to GNSS receiver */
    for (int i = 0; i < sizeof(encodedMessage); ++i) {
        uart_poll_out(zoe_uart_dev, encodedMessage[i]);
    }

    return 0;
}


/**
 * @brief Enable the NMEA ZDA sentence in the GPS output
 * 
 * @return 0 on success, else a negative error code
 */
static int32_t set_gnss_nmea_zda_enable(void)
{
    char message[3] = {0xf0, 0x08, 60};
    char encodedMessage[U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES + 3];
    int32_t ret;

    /* Encode the UBX-CFG-MSG message */
    ret = uUbxProtocolEncode(0x06, 0x01, message, sizeof(message), 
            encodedMessage);
    if (ret < 0) {
        LOG_ERR("set_gnss_nmea_zda_enable: Error encoding packet. (%d)",ret);
        return ret;
    }

    for (int i = 0; i < sizeof(encodedMessage); ++i) {
        uart_poll_out(zoe_uart_dev, encodedMessage[i]);
    }

    return 0;
}


/**
 * @brief Puts the GNSS receiver to a software sleep for the given duration. 
 * 
 * @param duration The duration to sleep in ms. 0 is indefinite sleep.
 * @return 0 on success, else negative error code from u_error_common.h
 */
static int32_t sleep_gnss(uint32_t duration)
{
    char message[16] = {0};
    char encodedMessage[U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES + 16];
    int32_t ret;

    /* Save duration */
    memcpy(&message[4], &duration, sizeof(duration));

    /* Sets BACKUP flag */
    message[8] = 1 << 1;

    /* Sets wakeup source to UART RX pin */
    message[12] = 1 << 3;

    /* Encode the UBX-RXM-PMREQ message. */
    ret = uUbxProtocolEncode(0x02, 0x41, message, sizeof(message), 
            encodedMessage);
    if (ret < 0) {
        LOG_ERR("sleep_gnss: Error encoding packet. (%d)", ret);
        return ret;
    }

    /* Send UBX message to GNSS receiver */
    for (int i = 0; i < sizeof(encodedMessage); ++i) {
        uart_poll_out(zoe_uart_dev, encodedMessage[i]);
    }

    return 0;
}


/**
 * @brief Wakeup the GNSS receiver from a software sleep
 */
static inline void wakeup_gnss(void)
{
    uart_poll_out(zoe_uart_dev, 0xff);
}


/**
 * @brief Read characters from UART until line end is detected. Afterwards push 
 *        the data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(zoe_uart_dev)) {
        return;
    }

    while (uart_irq_rx_ready(zoe_uart_dev)) {

        uart_fifo_read(zoe_uart_dev, &c, 1);

        if ((c == '\n') && rx_buf_pos > 0) {
            /* terminate string */
            rx_buf[rx_buf_pos++] = c;
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}