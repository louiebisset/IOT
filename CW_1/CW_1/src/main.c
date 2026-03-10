/* CW 1 – Multithreaded Step 1 – FIXED */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>  

#include <SEGGER_SYSVIEW.h>

#define ABS(x) ((x) < 0 ? -(x) : (x))

typedef enum {
    STATE_NORMAL = 0,
    STATE_WARNING,
    STATE_FAULT
} system_state_t;

typedef struct {
    int16_t raw;
    int32_t mv;
    int16_t temp_centi;
    bool    valid;
} sample_t;

/* ===== Shared state ===== */

static system_state_t system_state      = STATE_NORMAL;
static int16_t        latest_temp_centi = 0;
static int32_t        latest_mv         = 0; 
static int32_t temp_sum_centi = 0;
static int16_t avg_temp_centi = 0;
static uint16_t valid_samples = 0;  
static int32_t decimate_sum   = 0;
static uint8_t decimate_count = 0;

/* ===== Rolling Average Settings ===== */

#define SAMPLE_RATE_MS     100
#define DECIMATE_FACTOR    10
#define AVG_WINDOW_SAMPLES 60

typedef struct {
    int16_t buffer[AVG_WINDOW_SAMPLES];
    int32_t sum_centi;
    int32_t decimate_sum;
    uint16_t index;
    uint16_t valid_samples;
    uint8_t  decimate_count;
} temp_avg_t;

static temp_avg_t temp_avg = {0};

#define SAMPLE_PERIOD_MS      100
#define TEMP_THRESHOLD_CENTI  2800

/* ===== LED ===== */

#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ===== ADC ===== */

static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf;
static bool    adc_setup_done;

/* ===== Synchronisation ===== */

K_MUTEX_DEFINE(state_mutex);

/* Queue: Acquisition -> Logic
 * FIX 2: Increased queue depth from 10 to 20 so that a temporarily slow
 * logic thread does not cause silent sample drops under normal burst load. */
K_MSGQ_DEFINE(sample_msgq, sizeof(sample_t), 20, 4); /* FIX 2 */

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

/* ===== Acquisition ===== */

static int acquire_sample(sample_t *s)
{
    int err;

    if (s == NULL) {
        return -EINVAL;
    }

    s->raw        = 0;
    s->mv         = 0;
    s->temp_centi = 0;
    s->valid      = false;

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
        .buffer      = &adc_buf,
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
    s->mv  = s->raw;

    err = adc_raw_to_millivolts_dt(&adc_channel, &s->mv);
    if (err < 0) {
        printk("mV conversion failed\n");
        return err;
    }

    s->temp_centi = (int16_t)((s->mv * 10) - 27315);

    if (s->mv < 0 || s->temp_centi < -4000 || s->temp_centi > 12500) {
        s->valid = false;
        return -ERANGE;
    }

    s->valid = true;

    return 0;
}

/* ===== Logic ===== */

static void process_sample(const sample_t *s)
{
    if (s == NULL) {
        return;
    } 

    if (system_state == STATE_FAULT) {
        return;
    }
    if (!s->valid) {
        SEGGER_SYSVIEW_PrintfHost("State change: %s -> FAULT", state_to_string(system_state));
        system_state = STATE_FAULT;
        return;
    } 
    
    latest_temp_centi = s->temp_centi;
    latest_mv         = s->mv;

    /* Decimate: accumulate DECIMATE_FACTOR samples before adding to buffer */
    temp_avg.decimate_sum += s->temp_centi;
    temp_avg.decimate_count++;

    if (temp_avg.decimate_count >= DECIMATE_FACTOR) {
        int16_t decimated = (int16_t)(temp_avg.decimate_sum / DECIMATE_FACTOR);
        temp_avg.decimate_sum   = 0;
        temp_avg.decimate_count = 0;

        if (temp_avg.valid_samples < AVG_WINDOW_SAMPLES) {
            temp_avg.buffer[temp_avg.index] = decimated;
            temp_avg.sum_centi += decimated;
            temp_avg.valid_samples++;
        } else {
            temp_avg.sum_centi -= temp_avg.buffer[temp_avg.index];
            temp_avg.buffer[temp_avg.index] = decimated;
            temp_avg.sum_centi += decimated;
        }

        temp_avg.index = (temp_avg.index + 1) % AVG_WINDOW_SAMPLES;
    }

    if (temp_avg.valid_samples > 0) {
        avg_temp_centi = (int16_t)(temp_avg.sum_centi / (int32_t)temp_avg.valid_samples);
    }
}
/* ===== Reporting ===== */

// Reporting Function
static void report_status(void)
{
    int64_t now_ms = k_uptime_get();

    system_state_t st;
    int16_t avg_centi, latest_centi;
    int32_t mv;

    k_mutex_lock(&state_mutex, K_FOREVER);
    st           = system_state;
    avg_centi    = avg_temp_centi;
    latest_centi = latest_temp_centi;
    mv           = latest_mv;
    k_mutex_unlock(&state_mutex); 

   if (avg_temp_centi > TEMP_THRESHOLD_CENTI) {
        if (system_state != STATE_WARNING) {
            system_state = STATE_WARNING;
        }
    } else {
        if (system_state != STATE_NORMAL) {
            system_state = STATE_NORMAL;
        }
    }
}
/* ===== Threads ===== */

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

        /* FIX 2: Log a warning when the queue is full so drops are visible
         * rather than silently discarded. */
        if (k_msgq_put(&sample_msgq, &s, K_NO_WAIT) != 0) {
            printk("WARNING: sample queue full, sample dropped\n");
        }

        next += SAMPLE_PERIOD_MS;

        int64_t delay = next - k_uptime_get();
        if (delay < 0) delay = 0;

        k_sleep(K_MSEC(delay));
    }
}

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

void reporting_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {

        report_status();

        k_sleep(K_MSEC(1000));
    }
}

void led_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    /* FIX 4: Track the previous LED output level so that the FAULT branch
     * only calls gpio_pin_set_dt once (on the first entry) and then simply
     * sleeps, avoiding a redundant driver call every 200 ms when the state
     * cannot change. */
    int led_on = -1; /* unknown initial state */

    while (1) {

        system_state_t st;

        k_mutex_lock(&state_mutex, K_FOREVER);
        st = system_state;
        k_mutex_unlock(&state_mutex);

        if (st == STATE_NORMAL) {

            if (led_on != 0) {
                gpio_pin_set_dt(&led0, 0);
                led_on = 0;
            }
            k_sleep(K_MSEC(100));

        } else if (st == STATE_WARNING) {

            gpio_pin_set_dt(&led0, 1);
            led_on = 1;
            k_sleep(K_MSEC(50));

            gpio_pin_set_dt(&led0, 0);
            led_on = 0;
            k_sleep(K_MSEC(50));

        } else if (st == STATE_FAULT) {

            /* Solid ON: only drive the pin if not already on. */
            if (led_on != 1) {
                gpio_pin_set_dt(&led0, 1);
                led_on = 1;
            }
            k_sleep(K_MSEC(200));

        } else {

            k_sleep(K_MSEC(100));
        }
    }
}

/* ===== Thread Definitions ===== */

#define STACK_SIZE 1024
#define ACQ_PRIO   1
#define LOGIC_PRIO 2
#define LED_PRIO   3
#define REP_PRIO   4

K_THREAD_DEFINE(acq_tid,   STACK_SIZE, acquisition_thread,
                NULL, NULL, NULL, ACQ_PRIO,   0, 0);

K_THREAD_DEFINE(logic_tid, STACK_SIZE, logic_thread,
                NULL, NULL, NULL, LOGIC_PRIO, 0, 0);

K_THREAD_DEFINE(led_tid,   STACK_SIZE, led_thread,
                NULL, NULL, NULL, LED_PRIO,   0, 0);

K_THREAD_DEFINE(rep_tid,   STACK_SIZE, reporting_thread,
                NULL, NULL, NULL, REP_PRIO,   0, 0);

/* ===== main ===== */

int main(void)
{
    int err;

    printk("CW1 Thermal Monitoring – Multithreaded\n");

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
        k_sleep(K_FOREVER);
    }
}