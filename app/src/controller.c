#include <zephyr.h>
#include <device.h>
#include <zephyr/drivers/uart.h>
#include <logging/log.h>

#include "file/file_common.h"
#include "file/file_init.h"


LOG_MODULE_REGISTER(controller);


/* Node of ZOE-M8Q's UART */
#define ZOE_UART    DT_NODELABEL(uart1)

#define CONTROLLER_STACK_SIZE  2048
#define CONTROLLER_PRIORITY    3

/* Maximum NMEA message size */
#define MSG_SIZE 256

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* UART device for ZOE-M8Q GPS */
static const struct device *zoe_uart_dev = DEVICE_DT_GET(ZOE_UART);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
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


/**
 * @brief Increment the systems boot counter
 * 
 * @return 0 on success, else negative error code from FS API
 */
int increment_boot_count(void)
{
    uint32_t boot_count = 0;
    int err;

    err = read_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	if (err == -EEXIST) {
		boot_count = 0;
	} else if (err != 0) {
        return err;
    } else {
		LOG_INF("boot_count is: %d", boot_count);
		++boot_count;
	}

	err = write_file("boot_count", (uint8_t *)&boot_count, sizeof(boot_count));
	
    return err;
}


void controller_thread(void *a, void *b, void *c)
{
    char tx_buf[MSG_SIZE];


    LOG_INF("Controller started");

    #ifdef CONFIG_APP_INIT_BEACON_FILES
    initialise_files();
    #endif /* CONFIG_APP_INIT_BEACON_FILES */

    increment_boot_count();


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
            printk("%s", tx_buf);
        }
    }
}

K_THREAD_DEFINE(controller_id, CONTROLLER_STACK_SIZE, controller_thread, NULL, NULL, NULL, CONTROLLER_PRIORITY, 0, 0);