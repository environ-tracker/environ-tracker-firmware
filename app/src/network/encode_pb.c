#include <zephyr.h>
#include <posix/time.h>
#include <drivers/sensor.h>
#include <pb_encode.h>
#include <logging/log.h>

#include "src/network/proto/upload_data.pb.h"
#include "accumulator.h"

LOG_MODULE_DECLARE(lorawan_backend);

// static inline void save_sensor_val(SensorVal *buf, struct sensor_value *val)
// {
//     buf->val1 = val->val1;
//     buf->val2 = val->val2;
// }


bool encode_message(uint8_t *buffer, size_t buffer_size, size_t *message_len, 
        struct system_data *data)
{
    bool status;


    // EnvironTrackerUpload message = EnvironTrackerUpload_init_zero;
    SimpleMessage message = SimpleMessage_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);


    message.lucky_number = data->timestamp;

    // message.timestamp = data->timestamp;
    // LOG_INF("encode_message: Timestamp: %d", message.timestamp);

    // message.activity = data->activity;
    
    // message.has_location = true;
    // message.location.latitude = data->location.latitude;
    // message.location.longitude = data->location.longitude;
    // message.location.altitude = data->location.altitude;
    
    // message.has_environ_data = true;
    // message.environ_data.uv_index = data->environ.uv_index.val1;
    // save_sensor_val(&message.environ_data.ambient_temp, &data->environ.temp);
    // save_sensor_val(&message.environ_data.pressure, &data->environ.press);
    // save_sensor_val(&message.environ_data.humidity, &data->environ.humidity);
    // save_sensor_val(&message.environ_data.tvoc, &data->environ.gas_res);
    

    status = pb_encode(&stream, SimpleMessage_fields, &message);
    *message_len = stream.bytes_written;

    if (!status) {
        LOG_ERR("encode_message: Encoding failed: %s", PB_GET_ERROR(&stream));
    }

    return status;
}