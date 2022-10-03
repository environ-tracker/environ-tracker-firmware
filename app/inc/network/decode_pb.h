#ifndef DECODE_PB_H
#define DECODE_PB_H

#include <stdbool.h>
#include <stdint.h>

#include "ble/ble_network.h"

/**
 * @brief Decode and ibeacon request response message
 * 
 * @param ibeacon Storage for received ibeacon data
 * @param buffer Received message
 * @param message_length Length of received message
 * @return true If the message was correctly decoded
 * @return false If the message was not correctly decoded
 */
bool decode_ibeacon_request_response(struct ibeacon *ibeacon, uint8_t *buffer, 
        size_t message_length);

#endif /* DECODE_PB_H */