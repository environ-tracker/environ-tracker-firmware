#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "gnss/gnss.h"
#include "gnss/minmea.h"
#include "accumulator.h"
#include "location.h"


LOG_MODULE_REGISTER(gnss, CONFIG_LOG_DEFAULT_LEVEL);


/* Defines for the GNSS thread stack size and priority */
#define GNSS_STACK_SIZE 1024
#define GNSS_PRIORITY   5

/* Queue to store GNSS receiver messages */
K_MSGQ_DEFINE(uart_msgq, MINMEA_MAX_SENTENCE_LENGTH, 10, 4);

/* UART device for ZOE-M8Q GPS */
static const struct device *zoe_uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

/* Receive buffer used in UART ISR callback */
static char rx_buf[MINMEA_MAX_SENTENCE_LENGTH];
static int rx_buf_pos;

/* Controls behaviour of the GNSS module */
K_EVENT_DEFINE(gnss_control_events);


/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
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

void gnss_thread(void *a, void *b, void *c)
{
	struct location_wrapper location = {0};
    char tx_buf[MINMEA_MAX_SENTENCE_LENGTH];
	bool debug_output_enable = false;


    LOG_INF("GNSS thread started");

	/* Check if GNSS receiver UART is enabled */
    if (!device_is_ready(zoe_uart_dev)) {
		LOG_ERR("%s: GNSS UART device not ready.", zoe_uart_dev->name);
		return;
	}

    /* Configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(zoe_uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(zoe_uart_dev);


	location.source = LOCATION_GNSS;


    LOG_INF("ZOE-M8Q UART setup");

    while (1) {
		/* Check if debug output enable should be toggled */
		if (k_event_wait(&gnss_control_events, GNSS_TOGGLE_DEBUG_OUTPUT, 
				false, K_NO_WAIT)) {
			debug_output_enable = !debug_output_enable;

			/* Clear the event */
			k_event_set_masked(&gnss_control_events, 0, 
					GNSS_TOGGLE_DEBUG_OUTPUT);
		}

        while (k_msgq_get(&uart_msgq, &tx_buf, K_MSEC(50)) == 0) {
			if (debug_output_enable) {
				printk("%s", tx_buf);
			}
            

			// TODO: parse NMEA sentences

			switch (minmea_sentence_id(tx_buf, false)) {
			case MINMEA_SENTENCE_GGA:
				struct minmea_sentence_gga frame;

				if (!minmea_parse_gga(&frame, tx_buf)) {
					LOG_WRN("Issue parsing NMEA GGA message");
					break;	
				} else if (frame.fix_quality == 0) {
					break;
				}
				
				LOG_INF("xxGGA: fix quality: %d", frame.fix_quality);

				// TODO: store GGA data


				/* Place the retrieved location data on the queue */
				while (k_msgq_put(&location_msgq, &location, K_NO_WAIT) != 0) {
					k_msgq_purge(&location_msgq);
					LOG_DBG("location_msgq has been purged");
				}

				k_event_post(&data_events, LOCATION_DATA_PENDING);

				break;
			default:
				/* We don't care about these messages */
			}
        }
    }
}

K_THREAD_DEFINE(gnss_id, GNSS_STACK_SIZE, gnss_thread, NULL, NULL, NULL, 
        GNSS_PRIORITY, 0, 0);