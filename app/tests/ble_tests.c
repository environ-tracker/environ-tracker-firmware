#include <zephyr.h>
#include <stdio.h>
#include <bluetooth/uuid.h>
#include <fs/fs.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(tests);

#include "ble/ble_network.h"
#include "file/file_common.h"


void test_ble(void)
{
    struct bt_uuid_128 test_network = BT_UUID_INIT_128(BT_UUID_128_ENCODE(
            0xffaf1b67, 0xf3c0, 0x4a54, 0xb314, 0x58fff1960aff));

    char test_network_file[FILE_NAME_LEN];
    int position;

    // SETUP
    LOG_INF("Starting setup");

    strcpy(test_network_file, "beacons/");
    position = strlen(test_network_file);
    
    bt_uuid_to_str(&test_network.uuid, &test_network_file[position], 
            FILE_NAME_LEN - position);


    struct ibeacon_file_info data[10];
    int counter = 0;

    for (uint16_t major = 0; major <= 1; ++major) {
        for (uint16_t minor = 0; minor <= 4; ++minor) {
            data[counter].id = BEACON_ID_INIT(major, minor);
            data[counter].location.latitude = counter * 10;
            data[counter].location.longitude = counter * 100;
            data[counter].location.altitude = counter;
            ++counter;
        }
    }

    int ret = write_file(test_network_file, (uint8_t *)data, sizeof(data));
    if (ret) {
        LOG_ERR("setup: failed creating file. (%d)", ret);
        return;
    }

    LOG_WRN("Starting tests");

    // TEST: Test if supported network function works
    bool supported = is_supported_network(&test_network.uuid);
    if (supported) {
        LOG_WRN("supported_network: test passed");
    } else {
        LOG_ERR("supported_network: test failed");
    }

    // TEST: Test if supported network function works. See if cache works
    supported = is_supported_network(&test_network.uuid);
    if (supported) {
        LOG_WRN("supported_network: test passed");
    } else {
        LOG_ERR("supported_network: test failed");
    }

    // TEST: Test finding beacon
    struct ibeacon test_beacon = {
        .network_uuid = test_network,
        .id = data[0].id,
        .location = {0}
    };

    ret = find_beacon(&test_beacon);
    if (ret) {
        LOG_ERR("find_beacon: test failed (%d)", ret);
    } else {
        if (test_beacon.location.latitude == data[0].location.latitude &&
                test_beacon.location.longitude == data[0].location.longitude &&
                test_beacon.location.altitude == data[0].location.altitude) {
            LOG_WRN("find_beacon: test passed");
        } else {
            LOG_ERR("find_beacon: returned 0 but with incorrect location data");
        }
    }


    // TEST: Test finding beacon
    struct ibeacon test_beacon2 = {
        .network_uuid = test_network,
        .id = data[8].id,
        .location = {0}
    };

    ret = find_beacon(&test_beacon2);
    if (ret) {
        LOG_ERR("find_beacon: test failed (%d)", ret);
    } else {
        if (test_beacon2.location.latitude == data[8].location.latitude &&
                test_beacon2.location.longitude == data[8].location.longitude &&
                test_beacon2.location.altitude == data[8].location.altitude) {
            LOG_WRN("find_beacon: test passed");
        } else {
            LOG_ERR("find_beacon: returned 0 but with incorrect location data");
        }
    }

    // TEARDOWN
    ret = delete_file(test_network_file);
    if (ret) {
        LOG_ERR("teardown: failed deleting file. (%d)", ret);
        return;
    }
}

K_THREAD_DEFINE(tests, 2048, test_ble, NULL, NULL, NULL, 7, 0, 0);