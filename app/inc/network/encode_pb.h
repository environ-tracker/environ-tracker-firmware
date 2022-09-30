#ifndef ENCODE_PB_H
#define ENCODE_PB_H

#include <stdbool.h>
#include <stdint.h>

#include "accumulator.h"

/**
 * @brief Encode a system_data struct into a EnvironTrackerUpload protocol 
 *        buffer.
 * 
 * @param buffer Storage for encoded protocol buffer
 * @param buffer_size Size of buffer
 * @param message_len Size of the encoded message
 * @param data Data to encode into protocol buffer
 * @return true If protocol buffer is encoded successfully
 * @return false If protocol buffer isn't encoded successfully
 */
bool encode_message(uint8_t *buffer, size_t buffer_size, size_t *message_len,
        struct system_data *data);

#endif /* ENCODE_PB_H */