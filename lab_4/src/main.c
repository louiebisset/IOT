/* main.c - EEMEM0018 Lab Multi-Threaded Sensor Node (No Mutex) */

#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

/******************* Configuration Definitions ********************/
#define SAMPLE_INTERVAL_MS   1000   
#define REPORT_INTERVAL_MS   60000  
#define MY_GROUP_NUMBER      1      
#define DEVICE_NAME          "Louie Lab"
#define DEVICE_NAME_LEN      (sizeof(DEVICE_NAME) - 1)
#define COMPANY_ID           0x0059 
#define TEMP_THRESHOLD_LIMIT 24.5f
/******************* Hardware Specifications ********************/
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/******************* BLE Data Structures ********************/
struct adv_mfg_data {
    uint16_t company_id;   
    uint8_t  group_id;     
    int16_t  temperature;  
    int32_t  voltage;      
} __packed;
typedef struct adv_mfg_data adv_mfg_data_t;

static struct adv_mfg_data adv_mfg_data = {
    .company_id = COMPANY_ID,
    .group_id = MY_GROUP_NUMBER,
};

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (uint8_t *)&adv_mfg_data, sizeof(adv_mfg_data)),
};

#define BT_ADV_INTERVAL 0x1F40

// Now it's time for advertisement parameters
static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	BT_LE_ADV_OPT_NONE,
	BT_ADV_INTERVAL,
	BT_ADV_INTERVAL,
	NULL
);

/******************* Global Variables (Volatile for safety) ********************/
static int16_t adc_buf; 
static volatile float temp_history[60];
static volatile int sample_count = 0;
static volatile int32_t latest_voltage_mv = 0;
static bool adc_setup_done = false; 


/******************* Task 1: Data Processing (1Hz) ********************/
static void sample_work_handler(struct k_work *work)
{

    if (!adc_is_ready_dt(&adc_channel)) return;
    if (!adc_setup_done) {
        adc_channel_setup_dt(&adc_channel);
        adc_setup_done = true;
    }

    struct adc_sequence sequence = {
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
    };
    adc_sequence_init_dt(&adc_channel, &sequence);
    
    if (adc_read(adc_channel.dev, &sequence) == 0) {
        int32_t mv = (int32_t)adc_buf;
        adc_raw_to_millivolts_dt(&adc_channel, &mv);

        // Update globals directly
        latest_voltage_mv = mv;
        temp_history[sample_count % 60] = ((float)mv / 10.0f) - 273.15f;
        sample_count++;
    }
}
K_WORK_DEFINE(sample_worker, sample_work_handler);

/******************* Task 2: Reporting (1/60Hz) ********************/
/******************* Task 2: Communication Thread (1/60Hz) ********************/
static void report_work_handler(struct k_work *work)
{
    float sum = 0.0f;
    
    for (int i = 0; i < 60; i++) {
        sum += temp_history[i];
    }

    float avg_temp = sum / 60.0f;

    // --- ALERT LOGIC ---
    if (avg_temp > TEMP_THRESHOLD_LIMIT) { 
		gpio_pin_toggle_dt(&led);
        printk("!!! ALERT: TEMPERATURE EXCEEDED 28C (Current: %.2f C) !!!\n", (double)avg_temp);
    }

    int16_t temp_scaled = (int16_t)(avg_temp * 100);

    // Prepare data for transmission (Big Endian)
    adv_mfg_data.temperature = sys_cpu_to_be16(temp_scaled);
    adv_mfg_data.voltage = sys_cpu_to_be32(latest_voltage_mv);
    
    // Update the BLE packet
    bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);

    printk("[REPORTER] Min Avg: %d.%02d C | Voltage: %d mV\n", 
            temp_scaled / 100, abs(temp_scaled % 100), latest_voltage_mv);
}
K_WORK_DEFINE(report_worker, report_work_handler);


/******************* Timers ********************/
static void sample_timer_cb(struct k_timer *timer) { k_work_submit(&sample_worker); }
static void report_timer_cb(struct k_timer *timer) { k_work_submit(&report_worker); }

K_TIMER_DEFINE(sample_timer, sample_timer_cb, NULL);
K_TIMER_DEFINE(report_timer, report_timer_cb, NULL);

/******************* Main Entry ********************/
int main(void)
{
    printk("Starting EEMEM0018 Lab Node (No Mutex)...\n");

    if (!gpio_is_ready_dt(&led)) return -EIO;
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    if (bt_enable(NULL)) return -1;
    if (bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0)) return -1; 

	if (bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0)) {
        printk("Advertising failed to start\n");
        return -1;
    }

    k_timer_start(&report_timer, K_MSEC(REPORT_INTERVAL_MS), K_MSEC(REPORT_INTERVAL_MS));

    return 0;
}