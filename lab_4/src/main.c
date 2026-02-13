/* main.c - Application main entry point */

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
#define SAMPLE_PERIOD_MS 30000
// Get the ADC channel specification from the devicetree alias
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/******************* LED Configuration and Data Structures ********************/
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define GROUP_ID 0xFF 
/******************* BLE Configuration and Data Structures ********************/
#define COMPANY_ID 0x0059 
#define DEVICE_NAME "Louie_temp"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

struct adv_mfg_data {
    uint16_t company_id;
	uint8_t group_id; // Optional: Can be used to identify the type of data being sent
    int16_t temperature; // We'll store temp * 100 to keep precision in an int
	int16_t voltage; // Optional: If you want to include voltage or other sensor data

} __packed;
typedef struct adv_mfg_data adv_mfg_data_t;

static struct adv_mfg_data mfg_data = {
    .company_id = COMPANY_ID,
    .temperature = 0,
};

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, (uint8_t *)&mfg_data, sizeof(mfg_data)),
};

/******************* Workqueue and Timer ********************/
static void sample_work_handler(struct k_work *work);
K_WORK_DEFINE(sample_worker, sample_work_handler);

static void sample_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(sample_timer, sample_timer_handler, NULL);

/******************************************************************************/

static int16_t adc_buf;
static bool adc_setup_done = false;

static int read_sensor(int16_t *out_temp)
{
    int err;

    // LED ON: Starting measurement
    gpio_pin_set_dt(&led, 1);

    if (!adc_is_ready_dt(&adc_channel)) {
        return -EIO;
    }

    if (!adc_setup_done) {
        err = adc_channel_setup_dt(&adc_channel);
        if (err < 0) return err;
        adc_setup_done = true;
    }

    struct adc_sequence sequence = {
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
    };
    adc_sequence_init_dt(&adc_channel, &sequence);

    err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) return err;

    int32_t mv = (int32_t)adc_buf;
    adc_raw_to_millivolts_dt(&adc_channel, &mv);

    // Conversion: Change this formula based on your specific thermistor/sensor
    // Example: (mV - 500) / 10 for a typical analog temp sensor
    float temp_c = ((float)mv / 10.0f); 
    
    *out_temp = (int16_t)(temp_c * 100); // Store with 2 decimal places fixed-point

    // LED OFF: Success
    gpio_pin_set_dt(&led, 0);
    return 0;
}

static void sample_work_handler(struct k_work *work)
{
    int16_t temp;
    int err = read_sensor(&temp);

    if (err == 0) {
        mfg_data.temperature = temp;
        // Update the advertising data dynamically
        bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        printk("Updated Temp: %d.%02d C\n", temp / 100, abs(temp % 100));
    } else {
        printk("Sensor read failed: %d. LED remains ON.\n", err);
        // Note: LED stays ON because we don't call gpio_pin_set_dt(&led, 0) on error
    }
}

static void sample_timer_handler(struct k_timer *timer)
{
    // Submit work to the system queue (cannot do ADC/Logging directly in ISR)
    k_work_submit(&sample_worker);
}

/******************************************************************************/

int main(void)
{
    int err;

    printk("Starting LED/ADC/BLE Advertiser Demo\n");

    /* Initialize LED0 */
    if (!gpio_is_ready_dt(&led)) {
        return -EIO;
    }
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    /* Start Advertising */
    err = bt_le_adv_start(BT_LE_ADV_NCONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return err;
    }

    /* Start the periodic timer (starts immediately, repeats every 30s) */
    k_timer_start(&sample_timer, K_NO_WAIT, K_MSEC(SAMPLE_PERIOD_MS));

    return 0;
}