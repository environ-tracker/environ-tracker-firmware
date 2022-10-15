#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <pb_encode.h>
#include <zephyr/logging/log.h>

#include "src/network/proto/upload_data.pb.h"
// #include "src/network/proto/beacon.pb.h"
#include "accumulator.h"
#include "location.h"
#include "ble_network.h"
#include "environ.h"

LOG_MODULE_DECLARE(lorawan_backend);


/**
 * @brief Encodes system data into a protobuf
 * 
 * @param buffer Buffer to store the encoded protobuf
 * @param buffer_size Size of protobuf buffer
 * @param message_len Size of the encoded message
 * @param data System data to encode
 * @return true If the encoding was successful
 * @return false If an error occured while encoding
 */
bool encode_sys_data_message(uint8_t *buffer, size_t buffer_size, 
        size_t *message_len, struct system_data *data)
{
    struct location *loc = &data->location.location;
    struct environ_data *env = &data->environ;

    EnvironTrackerUpload message = EnvironTrackerUpload_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);

    message.activity = data->activity;
    
    message.has_location = true;
    message.location.location_source = data->location.source;
    message.location.latitude = location_float_to_int32(loc->latitude);
    message.location.longitude = location_float_to_int32(loc->longitude);
    message.location.altitude = (data->location.location.altitude * 100);

    message.has_environ = true;
    message.environ.uv_index = data->environ.uv_index.val1;
    message.environ.ambient_temp = sensor_value_to_double(&env->temp) * 100;
    message.environ.humidity = sensor_value_to_double(&env->humidity) * 100;
    message.environ.pressure = sensor_value_to_double(&env->press) * 1000;


    bool status = pb_encode(&stream, EnvironTrackerUpload_fields, &message);
    *message_len = stream.bytes_written;

    if (!status) {
        LOG_ERR("encode_message: Encoding failed: %s", PB_GET_ERROR(&stream));
    }

    return status;
}

// TODO: implement
bool encode_ibeacon_request_message(uint8_t *buffer, size_t buffer_size,
        size_t *message_len, struct ibeacon *data)
{
    return false;   
}