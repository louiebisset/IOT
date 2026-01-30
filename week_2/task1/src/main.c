/* main.c - Lab Activity 2 - Task 1 Template
 *
 * Task 1:
 *   - Complete conversion inside read_and_print_temperature()
 *     raw -> millivolts -> temperature (°C)
 *
 * Task 2 (optional):
 *   - If Temp > 30°C => LED ON, else OFF
 *
 * NOTE:
 *   This is a STUDENT TEMPLATE. Sections marked TODO must be completed.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

/* ===== Sampling period (Task 1 baseline) ===== */
#define SAMPLE_PERIOD_MS 500

/* ===== LED (Task 2 optional) ===== */
#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#endif
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define TEMP_THRESHOLD_C 28.0f

/* ===== ADC channel from devicetree =====
 * / {
 *   zephyr,user {
 *      io-channels = <&adc 0>;
 *   };
 * };
 */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf;
static bool adc_setup_done;

/* =========================================================
 * read_and_print_temperature()
 *  - reads ADC once
 *  - converts raw -> mV -> temperature
 *  - prints values
 *  - (optional) controls LED based on threshold
 * ========================================================= */
static int read_and_print_temperature(float *out_temp_c)
{
	int err;

	if (!adc_is_ready_dt(&adc_channel)) {
		printk("ADC %s is not ready\n", adc_channel.dev->name);
		return -EIO;
	}

	/* Setup ADC once */
	if (!adc_setup_done) {
		err = adc_channel_setup_dt(&adc_channel);
		if (err < 0) {
			printk("ADC channel setup failed (err=%d)\n", err);
			return err;
		}
		adc_setup_done = true;
	}

	struct adc_sequence sequence = {
		.buffer = &adc_buf,
		.buffer_size = sizeof(adc_buf),
	};

	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		printk("ADC sequence init failed (err=%d)\n", err);
		return err;
	}

	err = adc_read(adc_channel.dev, &sequence);
	if (err < 0) {
		printk("ADC read failed (err=%d)\n", err);
		return err;
	}

	int16_t raw = adc_buf;
	printk("ADC raw = %d\n", raw);

	/* ====== Task 1: convert raw -> millivolts and print ======
	 * TODO:
	 *   1) Create an int32_t mv
	 *   2) Use adc_raw_to_millivolts_dt(&adc_channel, &mv)
	 *   3) Print mv
	 */
	int32_t mv = raw; /* TODO: students set mv based on raw */
	err = adc_raw_to_millivolts_dt(&adc_channel, &mv);
    if (err < 0) {
        printk("Millivolts conversion failed\n");
        return err;
    }
	/* TODO: students call adc_raw_to_millivolts_dt() here */
	printk("ADC voltage = %d mV (TODO)\n", mv);

	/* ====== Task 1: convert mv -> temperature (°C) and print ======
	 * TODO:
	 *   Use LM335 relationship (10 mV/K):
	 *     T(K)  = mv / 10
	 *     T(°C) = (mv / 10) - 273.15
	 */
	float temp_c = 0.0f;
	float temp_k = ((float)mv / 10.0f);
	temp_c = temp_k - 273.15f;
	 /* TODO: students compute temp_c */
	printk("Temp = %.2f C (TODO)\n", (double)temp_c);

	/* ====== Task 2 (optional): threshold LED control ======
	 * TODO:
	 *   bool led_on = (temp_c > TEMP_THRESHOLD_C);
	 *   gpio_pin_set_dt(&led0, led_on ? 1 : 0);
	 */
	/* TODO: students implement LED threshold logic */
	/* printk("LED = %s\n", led_on ? "ON" : "OFF"); */

	if (out_temp_c) {
		*out_temp_c = temp_c;
	}
	bool led_on = (temp_c > TEMP_THRESHOLD_C);
    
    /* Set the GPIO pin: 1 for High (ON), 0 for Low (OFF) */
    gpio_pin_set_dt(&led0, led_on ? 1 : 0);
	printk("LED = %s\n", led_on ? "ON" : "OFF");
	return 0;
}

int main(void)
{
	int err;

	printk("Lab Activity 2 - Task 1 Template\n");

	/* LED init (for Task 2 optional; safe to keep even if Task 2 not done yet) */
	if (!gpio_is_ready_dt(&led0)) {
		printk("LED device not ready\n");
		return 0;
	}
	err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		printk("LED configure failed (err=%d)\n", err);
		return 0;
	}

	while (1) {
		(void)read_and_print_temperature(NULL);
		k_sleep(K_MSEC(SAMPLE_PERIOD_MS));
	}
}
