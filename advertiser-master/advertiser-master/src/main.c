/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2025-2026 University of Bristol
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/byteorder.h>

// This is our device name from prj.conf
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// This is the company ID that will be in the Manufacturer Specific Data
#define COMPANY_ID 0x0059 // Nordic Semiconductor ASA

// Define Manufacturer Specific Data structure
struct adv_mfg_data {
	uint16_t company_id;
	int16_t temperature;
} __packed;
typedef struct adv_mfg_data adv_mfg_data_t;

/**
 * Our advertisement data structure.
 * 
 * We include the device name and the manufacturer specific data (which includes specific company ID and temperature sensor value).
 */
static const struct bt_data ad[] = {
};

/**
 * Define advertising interval (in units of 0.625 ms)
 * 
 * For example, 0x1F40 = 8000 * 0.625 ms = 5000 ms (5s)
 */
#define BT_ADV_INTERVAL 0x1F40

// Now it's time for advertisement parameters
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_NONE,
	BT_ADV_INTERVAL,
	BT_ADV_INTERVAL,
	NULL
);


int main(void)
{
	int err;

	printk("Starting BLE Advertiser Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");


	return 0;
}
