/* main.c - Lab Activity 2 - Task 5: Button-Triggered ADC
 *
 * Flow:
 * 1. Main thread sleeps waiting for a Semaphore (Button).
 * 2. User presses Button -> GPIO Interrupt -> Gives Semaphore.
 * 3. Main thread wakes up -> Calls adc_sample_async().
 * 4. ADC Hardware samples -> ADC Interrupt -> Signals completion.
 * 5. Main thread prints result and goes back to sleep.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

#define TEMP_THRESHOLD_C 30.0f

/* ===== 1. Hardware Definitions ===== */

/* LED */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Button (sw0) */
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* ADC channel */
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* ===== 2. Synchronization Objects ===== */

static int16_t adc_buf;
static bool adc_setup_done;

/* Semaphore to signal "Button was pressed" */
K_SEM_DEFINE(button_sem, 0, 1);

/* Async ADC objects (from Task 4) */
static struct k_poll_signal adc_sig;
static struct k_poll_event  adc_evt = K_POLL_EVENT_INITIALIZER(
    K_POLL_TYPE_SIGNAL,
    K_POLL_MODE_NOTIFY_ONLY,
    &adc_sig
);

/* ===== 3. Interrupt Handlers ===== */

/* GPIO Interrupt Service Routine (Runs when button is pressed) */
static void button_pressed_handler(const struct device *dev, struct gpio_callback *cb,
                                   uint32_t pins)
{
    /* IMPORTANT: Do not run ADC code here! 
     * ISRs must be fast. We just "give" the semaphore to wake up the main thread. 
     */
    k_sem_give(&button_sem);
}

/* Struct to hold callback data */
static struct gpio_callback button_cb_data;


/* ===== 4. ADC Function (Same as Task 4) ===== */
static int adc_sample_async(int16_t *out_raw)
{
    int err;
    if (!adc_is_ready_dt(&adc_channel)) return -EIO;

    if (!adc_setup_done) {
        adc_channel_setup_dt(&adc_channel);
        adc_setup_done = true;
    }

    struct adc_sequence seq = {
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
    };

    adc_sequence_init_dt(&adc_channel, &seq);
    k_poll_signal_reset(&adc_sig);

    /* Start async read */
    err = adc_read_async(adc_channel.dev, &seq, &adc_sig);
    if (err < 0) return err;

    /* Wait for ADC completion */
    return k_poll(&adc_evt, 1, K_MSEC(1000));
}

/* ===== 5. Main Loop ===== */
int main(void)
{
    int err;
    printk("Press Button 1 (sw0) to read Temperature...\n");

    /* --- Init LED --- */
    if (gpio_is_ready_dt(&led0)) {
        gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    }

    /* --- Init ADC Signal --- */
    k_poll_signal_init(&adc_sig);

    /* --- Init Button & Interrupt --- */
    if (!gpio_is_ready_dt(&button)) {
        printk("Button device not ready\n");
        return 0;
    }
    /* Configure pin as input */
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    
    /* Configure interrupt: Edge Rising means "when pressed" (usually) */
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);

    /* Setup the callback function */
    gpio_init_callback(&button_cb_data, button_pressed_handler, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);


    while (1) {
        /* * WAIT HERE FOREVER! 
         * The CPU will sleep until 'button_sem' is given by the ISR.
         */
        k_sem_take(&button_sem, K_FOREVER);

        printk("Button pressed! Sampling ADC...\n");

        int16_t raw = 0;
        err = adc_sample_async(&raw);

        if (err == 0) {
            /* Reuse Task 1/2 logic */
            int32_t mv = raw; /* Raw is in buffer after adc_sample_async returns */
            adc_buf = raw; /* Ensure buffer sync if needed, though raw is copied */
            
            /* Zephyr internal conversion */
            adc_raw_to_millivolts_dt(&adc_channel, &mv);
            
            /* Calculate Temp */
            float temp_c = ((float)mv / 10.0f) - 273.15f;

            /* LED Logic */
            bool led_on = (temp_c > TEMP_THRESHOLD_C);
            gpio_pin_set_dt(&led0, led_on ? 1 : 0);

            printk("Temp: %.2f C | LED: %s\n", (double)temp_c, led_on ? "ON" : "OFF");
        } else {
            printk("ADC failed: %d\n", err);
        }
    }
}