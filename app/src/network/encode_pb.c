#include <zephyr.h>
#include <drivers/sensor.h>
#include <pb_encode.h>
#include <logging/log.h>

#include "src/network/proto/upload_data.pb.h"
#include "accumulator.h"

LOG_MODULE_DECLARE(lorawan_backend);


bool encode_message(uint8_t *buffer, size_t buffer_size, size_t *message_len, 
        struct system_data *data)
{
    bool status;


    // EnvironTrackerUpload message = EnvironTrackerUpload_init_zero;
    EnvironTrackerUpload message = EnvironTrackerUpload_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);

    message.activity = data->activity;
    
    message.has_location = true;
    message.location.latitude = data->location.latitude;
    message.location.longitude = data->location.longitude;
    message.location.altitude = data->location.altitude;
    
    // TODO: Correctly format data here
    message.has_environ = true;
    message.environ.uv_index = data->environ.uv_index.val1;
    message.environ.ambient_temp = data->environ.temp.val1;
    message.environ.humidity = data->environ.humidity.val1;
    message.environ.pressure = data->environ.press.val1;


    status = pb_encode(&stream, EnvironTrackerUpload_fields, &message);
    *message_len = stream.bytes_written;

    if (!status) {
        LOG_ERR("encode_message: Encoding failed: %s", PB_GET_ERROR(&stream));
    }

    return status;
}