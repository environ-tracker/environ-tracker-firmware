#include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(controller);

#define CONTROLLER_STACK_SIZE  2048
#define CONTROLLER_PRIORITY    3



void controller_thread(void *a, void *b, void *c)
{

}

// K_THREAD_DEFINE(controller_id, CONTROLLER_STACK_SIZE, controller_thread, NULL, NULL, NULL, CONTROLLER_PRIORITY, 0, 0);