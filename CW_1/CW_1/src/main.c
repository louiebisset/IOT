/* CW 1 – Multithreaded Step 1 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

typedef enum {
    STATE_NORMAL = 0,
    STATE_WARNING,
    STATE_FAULT
} system_state_t;

typedef struct {
    int16_t raw;
    int32_t mv;
    float   temp_c;
    bool    valid;
} sample_t;

/* ===== Shared state (protected by mutex) ===== */
static system_state_t system_state = STATE_NORMAL;
static float latest_temp_c = 0.0f;
static int32_t latest_mv = 0;

// Stores exact 1 min rolling average using fixed-point centi-degC
#define SAMPLE_RATE_MS        100     // 100ms hardware sample rate
#define DECIMATE_FACTOR       10      // store 1 in every 10 samples = 1s
#define AVG_WINDOW_SAMPLES    60      // 60 × 1s = 60 seconds

typedef struct {
    int16_t  buffer[AVG_WINDOW_SAMPLES];  
    int32_t  sum_centi;                 
    int32_t  decimate_sum;                
    uint16_t index;                       . 
    uint8_t  valid_samples;              
    uint8_t  decimate_count;              
} temp_avg_t;

#define SAMPLE_PERIOD_MS 100
#define TEMP_THRESHOLD_C 28.0f

/* LED */
#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ADC */
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf;
static bool adc_setup_done;

/* Mutex for shared state */
K_MUTEX_DEFINE(state_mutex);

/* Message queue: samples from Acquisition -> Logic */
K_MSGQ_DEFINE(sample_msgq, sizeof(sample_t), 10, 4);

/* ===== Utility ===== */

static const char *state_to_string(system_state_t state)
{
    switch (state) {
    case STATE_NORMAL:  return "NORMAL";
    case STATE_WARNING: return "WARNING";
    case STATE_FAULT:   return "FAULT";
    default:            return "UNKNOWN";
    }
}

static const char *led_to_string(system_state_t state)
{
    switch (state) {
    case STATE_NORMAL:  return "OFF";
    case STATE_WARNING: return "BLINKING";
    case STATE_FAULT:   return "SOLID";
    default:            return "UNKNOWN";
    }
}

/* ===== Acquisition Function (unchanged logic) ===== */

static int acquire_sample(sample_t *s)
{
    int err;

    if (s == NULL) {
        return -EINVAL;
    }

    s->raw = 0;
    s->mv = 0;
    s->temp_c = 0.0f;
    s->valid = false;

    if (!adc_is_ready_dt(&adc_channel)) {
        printk("ADC %s is not ready\n", adc_channel.dev->name);
        return -EIO;
    }

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

    s->raw = adc_buf;
    s->mv = s->raw;

    err = adc_raw_to_millivolts_dt(&adc_channel, &s->mv);
    if (err < 0) {
        printk("mV conversion failed\n");
        return err;
    }

    s->temp_c = ((float)s->mv / 10.0f) - 273.15f;

    if (s->mv < 0 || s->temp_c < -40.0f || s->temp_c > 125.0f) {
        s->valid = false;
        return -ERANGE;
    }

    s->valid = true;
    return 0;
}

/* ===== Logic Function (now called under mutex) ===== */

static temp_avg_t temp_avg = {0};  // replaces all scattered globals

static void process_sample(const sample_t *s)
{
    if (s == NULL) {
        return;
    }
    if (system_state == STATE_FAULT) {
        return;
    }
    if (!s->valid) {
        system_state = STATE_FAULT;
        return;
    }

    latest_temp_centi = s->temp_centi;
    latest_mv         = s->mv;

    // Update rolling 1-minute average (decimated)
    temp_avg_update(&temp_avg, s->temp_centi);

    // Update system state
    int16_t avg = temp_get_avg(&temp_avg);
    if (avg > TEMP_THRESHOLD_CENTI) {
        system_state = STATE_WARNING;
    } else {
        system_state = STATE_NORMAL;
    }
}

/* ===== Reporting Function (reads shared state under mutex) ===== */

static void report_status(void)
{
    int64_t now_ms = k_uptime_get();

    system_state_t st;
    float avg, latest;
    int32_t mv;

    k_mutex_lock(&state_mutex, K_FOREVER);
    st     = system_state;
    avg    = avg_temp_c;
    latest = latest_temp_c;
    mv     = latest_mv;
    k_mutex_unlock(&state_mutex);

    if (st == STATE_FAULT) {
        printk("[%lld ms] Avg: --.-C | Raw: %d mV | Mode: %s | LED: %s\n",
               now_ms,
               mv,
               state_to_string(st),
               led_to_string(st));
    } else {
        printk("[%lld ms] Avg: %.2fC | Raw: %.2fC | Mode: %s | LED: %s\n",
               now_ms,
               (double)avg,
               (double)latest,
               state_to_string(st),
               led_to_string(st));
    }
}

/* ===== Threads ===== */

/* Acquisition thread: highest priority, 100 ms periodic */
void acquisition_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    int64_t next = k_uptime_get();

    while (1) {
        sample_t s = {0};
        int err = acquire_sample(&s);
        if (err < 0) {
            s.valid = false;
        }

        /* Send sample to logic thread */
        (void)k_msgq_put(&sample_msgq, &s, K_NO_WAIT);

        next += SAMPLE_PERIOD_MS;
        int64_t now = k_uptime_get();
        int64_t delay = next - now;
        if (delay < 0) {
            delay = 0;
        }
        k_sleep(K_MSEC(delay));
    }
}

/* Logic thread: consumes samples, updates shared state under mutex */
void logic_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    sample_t s;

    while (1) {
        k_msgq_get(&sample_msgq, &s, K_FOREVER);

        k_mutex_lock(&state_mutex, K_FOREVER);
        process_sample(&s);
        k_mutex_unlock(&state_mutex);
    }
}

/* Reporting thread: periodic logging */
void reporting_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        report_status();
        k_sleep(K_MSEC(1000)); /* e.g. log every 500 ms */
    }
}

/* LED thread: handles OFF / BLINKING / SOLID */
void led_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        system_state_t st;

        k_mutex_lock(&state_mutex, K_FOREVER);
        st = system_state;
        k_mutex_unlock(&state_mutex);

        if (st == STATE_NORMAL) {
            gpio_pin_set_dt(&led0, 0);
            k_sleep(K_MSEC(100));
        } else if (st == STATE_WARNING) {
            gpio_pin_set_dt(&led0, 1);
            k_sleep(K_MSEC(50));
            gpio_pin_set_dt(&led0, 0);
            k_sleep(K_MSEC(50));
        } else if (st == STATE_FAULT) {
            gpio_pin_set_dt(&led0, 1);
            k_sleep(K_MSEC(200));
        } else {
            k_sleep(K_MSEC(100));
        }
    }
}

/* ===== Thread definitions ===== */

#define STACK_SIZE 1024
#define ACQ_PRIO   1
#define LOGIC_PRIO 2
#define LED_PRIO   3
#define REP_PRIO   4

K_THREAD_DEFINE(acq_tid,   STACK_SIZE, acquisition_thread, NULL, NULL, NULL, ACQ_PRIO,   0, 0);
K_THREAD_DEFINE(logic_tid, STACK_SIZE, logic_thread,       NULL, NULL, NULL, LOGIC_PRIO, 0, 0);
K_THREAD_DEFINE(led_tid,   STACK_SIZE, led_thread,         NULL, NULL, NULL, LED_PRIO,   0, 0);
K_THREAD_DEFINE(rep_tid,   STACK_SIZE, reporting_thread,   NULL, NULL, NULL, REP_PRIO,   0, 0);

/* ===== main ===== */

int main(void)
{
    int err;

    printk("CW_1 Thermal Monitoring – Multithreaded Step 1\n");

    if (!gpio_is_ready_dt(&led0)) {
        printk("LED device not ready\n");
        return 0;
    }

    err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("LED configure failed (err=%d)\n", err);
        return 0;
    }

    /* Threads already started by K_THREAD_DEFINE */
    while (1) {
        k_sleep(K_FOREVER);
    }
}
