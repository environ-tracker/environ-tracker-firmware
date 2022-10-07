#include <zephyr/kernel.h>
#include <pb_decode.h>
#include <zephyr/logging/log.h>

#include "src/network/proto/beacon.pb.h"
#include "ble/ble_network.h"

bool decode_ibeacon_request_response(struct ibeacon *ibeacon, uint8_t *buffer, 
        size_t message_length)
{
    BeaconResponse response = BeaconResponse;

    pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);

    bool status = pb_decode(&stream, BeaconResponse_fields, &response);
    if (status) {

        ibeacon->id = response.id;
        // TODO: Handle network uuid
        ibeacon->location.latitude = response.location.latitude;
        ibeacon->location.longitude = response.location.longitude;
        ibeacon->location.altitude = response.location.altitude;

    } else {
        LOG_ERR("decode_ibeacon_request_response: Decoding failed: %s", 
                PB_GET_ERROR(&stream));
    }

    return status;
}