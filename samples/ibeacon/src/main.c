/*
 * Copyright (c) 2018 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>


#define IBEACON_UUID_1	0x7a,0xaf,0x1b,0x67
#define IBEACON_UUID_2	0xf3,0xc0
#define IBEACON_UUID_3	0x4a,0x54
#define IBEACON_UUID_4	0xb3,0x14
#define IBEACON_UUID_5	0x58,0xff,0xf1,0x96,0x0a,0x40

#ifndef IBEACON_MAJOR
#define IBEACON_MAJOR	0x00,0x00
#endif

#ifndef IBEACON_MINOR
#define IBEACON_MINOR	0x00,0x00
#endif

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
#endif

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  18ee1516-016b-4bec-ad96-bcb96d166e97
 * Major: 0
 * Minor: 0
 * RSSI:  -56 dBm
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		      0x00, 0x4c, /* Apple */
		      0x02, 0x15, /* iBeacon */
		      IBEACON_UUID_1, /* UUID[15..12] */
		      IBEACON_UUID_2, /* UUID[11..10] */
		      IBEACON_UUID_3, /* UUID[9..8] */
		      IBEACON_UUID_4, /* UUID[7..6] */
		      IBEACON_UUID_5, /* UUID[5..0] */
			  IBEACON_MAJOR, /* Major */
		      IBEACON_MINOR, /* Minor */
		      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
			      NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("iBeacon started\n");
}

void main(void)
{
	int err;

	printk("Starting iBeacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
