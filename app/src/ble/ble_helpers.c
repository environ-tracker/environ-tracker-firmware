#include <string.h>
#include <sys/util.h>
#include <bluetooth/uuid.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_helpers);

#include "ble/ble_helpers.c"


int bt_uuid_from_str(char *uuid, struct bt_uuid *out)
{
    uint8_t buf[BT_UUID_SIZE_128], temp[2];
    bool ret;
    int j = 0, err;

    // TODO: strnlen
    const int uuid_len = strlen(uuid);
    if (uuid_len % 2 != 0) {
        return -EINVAL;
    }

    for (int i = BT_UUID_SIZE_128 - 1; i >= 0; --i) {
        for (int k = 0; k < 2; ++k) {
            if (uuid[j + k] == '-') {
                j++;
            }

            err = char2hex(uuid[j + k], &temp[k]);
            if (err) {
                return err;
            }
        }
        buf[i] = temp[0] << 4 | temp[1];
        j += 2;
    }

    ret = bt_uuid_create(out, buf, BT_UUID_SIZE_128);

    // TODO: better negative return code
    return (ret) ? 0 : -1; 
}