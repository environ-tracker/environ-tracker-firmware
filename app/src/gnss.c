#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <logging/log.h>


LOG_MODULE_REGISTER(gnss, CONFIG_LOG_DEFAULT_LEVEL);


/* Defines for the GNSS thread stack size and priority */
#define GNSS_STACK_SIZE 1024
#define GNSS_PRIORITY   5

/* Node of ZOE-M8Q's UART */
#define ZOE_UART    DT_NODELABEL(uart1)

/* Maximum NMEA message size */
#define NMEA_MAX_MSG_SIZE 256

/* Queue to store GNSS receiver messages */
K_MSGQ_DEFINE(uart_msgq, NMEA_MAX_MSG_SIZE, 10, 4);

/* UART device for ZOE-M8Q GPS */
static const struct device *zoe_uart_dev = DEVICE_DT_GET(ZOE_UART);

/* Receive buffer used in UART ISR callback */
static char rx_buf[NMEA_MAX_MSG_SIZE];
static int rx_buf_pos;


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
    char tx_buf[NMEA_MAX_MSG_SIZE];


    LOG_INF("GNSS thread started");


    if (!device_is_ready(zoe_uart_dev)) {
		printk("UART device not found!");
		return;
	}

    /* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(zoe_uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(zoe_uart_dev);


    LOG_INF("ZOE-M8Q UART setup");

    while (1) {
        while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
            // TODO: parse NMEA sentences

            printk("%s", tx_buf);
        }
    }
}

K_THREAD_DEFINE(gnss_id, GNSS_STACK_SIZE, gnss_thread, NULL, NULL, NULL, 
        GNSS_PRIORITY, 0, 0);