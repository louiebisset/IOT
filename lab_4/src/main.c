/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2025-2026 University of Bristol
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/******************************************************************************/
#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
/******************* ADC Configuration and Data Structures ********************/
/**
 * Use this block to configure the ADC / sensor reading worker
 * - Define the sampling period (30 sec)
 * - Define the ADC channel from device tree
 * - Define the sensor reading worker
 * - Define any other ADC-/Sensor-related variables that may be needed
 */
/******************* LED Configuration and Data Structures ********************/
/**
 * Use this block to configure the LED
 * - Define the LED Node alias
 * - Define the gpio device tree spec
 */
/******************* BLE Configuration and Data Structures ********************/
/**
 * Use this block to define BLE functionality
 * - Define the device name, configurable from prj.conf
 * - Define the COMPANY ID for the Manufacturer Specific Data element
 * - Define the format of the MSD payload, and a variable to hold it
 * - Define the format of your payload
 * - Define the advertising interval and other BLE parameters
 */
/******************************************************************************/
/**
 * Use this block to define the functions needed to read the sensor
 * - A function that reads the sensor
 *   The function must turn the LED on at the start, and turn it back off if
 *   it ends successfully. In this scenario, the user will see a quick blink.
 *   If an error occurs, the LED will remain on.
 * 
 * - The sensor reading worker.
 *   When the worker has read the sensor successfully, it must update the
 *   correct part of the BLE advertisement payload.
 */
/* 
 * Read ADC once, convert to temperature and populate the arguments
 */
static int read_sensor(float *out_temp_c, int32_t *out_voltage)
{
	return 0;
}
/* Runs in system workqueue thread context (safe for adc_read) */
static void sample_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
}
/* Runs in timer context: do NOT call adc_read here */
static void sample_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
}
/******************************************************************************/
int main(void)
{
	int err;

	printk("Starting LED/ADC/BLE Advertiser Demo\n");

	/* Initialize LED0 */

	/* Start the ADC periodic sampling worker */

	/* Initialize the Bluetooth Subsystem */

	return 0;
}
