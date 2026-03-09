* Design Challenge 1: High-Reliability Thermal Monitor and Controller */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <stdint.h>

#define ABS(x) ((x) < 0 ? -(x) : (x))
// State definitions
typedef enum {
    STATE_NORMAL = 0, // = Average temp <= threshold
    STATE_WARNING,    // = 1 min Average temp > threshold
    STATE_FAULT       // = Incorrect reading
} system_state_t;

//Sample Struct
typedef struct {
    int16_t raw;
    int32_t mv;
    int16_t temp_centi;
    bool    valid;
} sample_t;

// Shared states
static system_state_t system_state = STATE_NORMAL;
static int16_t latest_temp_centi = 0;
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

#define SAMPLE_PERIOD_MS 100    // Aquistion period
#define TEMP_THRESHOLD_CENTI 2800  // Temp warning thresh

//LED Setup
#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

//ADC Setup
static const struct adc_dt_spec adc_channel =
    ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static int16_t adc_buf;     // rAW Adc buff
static bool adc_setup_done;

K_MUTEX_DEFINE(state_mutex);
K_MSGQ_DEFINE(sample_msgq, sizeof(sample_t), 10, 4);
K_SEM_DEFINE(sample_sem, 0, 1);
static struct k_timer sample_timer;

// BLE Setup
#define DEVICE_NAME "QASIM"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define COMPANY_ID 0x0059
#define GROUP_ID   0x01
#define BLE_UPDATE_PERIOD_MS 1000
#define BT_ADV_INTERVAL 0x00A0  

//BLE data Structs
struct adv_mfg_data {
    uint16_t company_id;
    uint8_t  group_id;
    int16_t  avg_temp_x100;
    uint8_t  state;
} __packed;

static struct adv_mfg_data adv_mfg_data = {
    .company_id = COMPANY_ID,
    .group_id = GROUP_ID,
    .avg_temp_x100 = 0,
    .state = STATE_NORMAL,
};

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA,
            (uint8_t *)&adv_mfg_data,
            sizeof(adv_mfg_data)),
};

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    BT_LE_ADV_OPT_NONE,
    BT_ADV_INTERVAL,
    BT_ADV_INTERVAL,
    NULL
);

static bool ble_started = false;

//Utility Functions
static const char *state_to_string(system_state_t state)    //Converts enum state to print in serial
{
    switch (state) {
    case STATE_NORMAL:  return "NORMAL";
    case STATE_WARNING: return "WARNING";
    case STATE_FAULT:   return "FAULT";
    default:            return "UNKNOWN";
    }
}

static const char *led_to_string(system_state_t state)  //Converts state into diff LED modes
{
    switch (state) {
    case STATE_NORMAL:  return "OFF";
    case STATE_WARNING: return "BLINKING";
    case STATE_FAULT:   return "SOLID";
    default:            return "UNKNOWN";
    }
}

static void sample_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    k_sem_give(&sample_sem);
}

//Aquisition Function
static int acquire_sample(sample_t *s)
{
    int err;

    if (s == NULL) {
        return -EINVAL;
    }
    //Initialise samples to 0
    s->raw = 0;
    s->mv = 0;
    s->temp_centi = 0;
    s->valid = false;

    //Check ADC Ready
    if (!adc_is_ready_dt(&adc_channel)) {
        printk("ADC %s is not ready\n", adc_channel.dev->name);
        return -EIO;
    }

    if (!adc_setup_done) {              //ADC Setup
        err = adc_channel_setup_dt(&adc_channel);
        if (err < 0) {
            printk("ADC channel setup failed (err=%d)\n", err);
            return err;
        }
        adc_setup_done = true;
    }

    struct adc_sequence sequence = {    //Read config
        .buffer = &adc_buf,
        .buffer_size = sizeof(adc_buf),
    };

    err = adc_sequence_init_dt(&adc_channel, &sequence);       //Fill sequence
    if (err < 0) {
        printk("ADC sequence init failed (err=%d)\n", err);
        return err;
    }

    err = adc_read(adc_channel.dev, &sequence);     //ADC Conversion
    if (err < 0) {
        printk("ADC read failed (err=%d)\n", err);
        return err;
    }

    s->raw = adc_buf;  //Stores results
    s->mv = s->raw;

    err = adc_raw_to_millivolts_dt(&adc_channel, &s->mv);   //Converts from raw to mv
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

    if (st == STATE_FAULT) {
        printk("[%lld ms] Avg: --.-C | Voltage: %d mV | Mode: %s | LED: %s\n",
               now_ms,
               mv,
               state_to_string(st),
               led_to_string(st));
    } else {
        printk("[%lld ms] Avg: %d.%02dC | Latest: %d.%02dC | Mode: %s | LED: %s\n",
               now_ms,
               avg_centi / 100, ABS(avg_centi % 100),
               latest_centi / 100, ABS(latest_centi % 100),
               state_to_string(st),
               led_to_string(st));
    }
}

// Thread Functions
//BLE Thread- Updates the BLE payload with latest average temp and state
void ble_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        if (!ble_started) {
            k_sleep(K_MSEC(100));
            continue;
        }

        system_state_t st;
        int16_t avg_centi;
        int err;

        k_mutex_lock(&state_mutex, K_FOREVER);
        st = system_state;
        avg_centi = avg_temp_centi;
        k_mutex_unlock(&state_mutex);

        if (st == STATE_FAULT) {
            k_sleep(K_MSEC(BLE_UPDATE_PERIOD_MS));
            continue;
        }

        adv_mfg_data.avg_temp_x100 = sys_cpu_to_be16(avg_centi);
        adv_mfg_data.state = (uint8_t)st;

        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("BLE update failed (err=%d)\n", err);
        }

        k_sleep(K_MSEC(BLE_UPDATE_PERIOD_MS));
    }
}

// Aquisition Thread - Runs every 100ms- takes 1 sample- sends to logic thread
void acquisition_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        /* Wait until the 100 ms timer releases this thread */
        k_sem_take(&sample_sem, K_FOREVER);

        sample_t s = {0};
        int err = acquire_sample(&s);
        if (err < 0) {
            s.valid = false;
        }

        /* Send sample to logic thread */
        int q_err = k_msgq_put(&sample_msgq, &s, K_NO_WAIT);
        if (q_err != 0) {
            printk("sample_msgq put failed (err=%d)\n", q_err);
        }
    }
}

// logic Thread - waits for sample from aquisition thread- processes it- updates shared state
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

// Reporting Thread- Prints system status
void reporting_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        report_status();
        k_sleep(K_MSEC(1000)); // Reports Every second
    }
}

// LED Thread- Sets LED mode based on system state (off, blinking, solid)
void led_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    while (1) {
        system_state_t st;

        k_mutex_lock(&state_mutex, K_FOREVER);
        st = system_state;
        k_mutex_unlock(&state_mutex);

        if (st == STATE_NORMAL) {
            gpio_pin_set_dt(&led0, 0);      // OFF
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

// Thread Definitions - 5 is lowest priortiy 1 is highest
#define STACK_SIZE 1024
#define ACQ_PRIO   1    // Highest priortiy
#define LOGIC_PRIO 2
#define LED_PRIO   3
#define REP_PRIO   4
#define BLE_PRIO   5    // Lowest priority

K_THREAD_DEFINE(acq_tid,   STACK_SIZE, acquisition_thread, NULL, NULL, NULL, ACQ_PRIO,   0, 0);
K_THREAD_DEFINE(logic_tid, STACK_SIZE, logic_thread,       NULL, NULL, NULL, LOGIC_PRIO, 0, 0);
K_THREAD_DEFINE(led_tid,   STACK_SIZE, led_thread,         NULL, NULL, NULL, LED_PRIO,   0, 0);
K_THREAD_DEFINE(rep_tid,   STACK_SIZE, reporting_thread,   NULL, NULL, NULL, REP_PRIO,   0, 0);
K_THREAD_DEFINE(ble_tid,   STACK_SIZE, ble_thread,         NULL, NULL, NULL, BLE_PRIO,   0, 0);

//main
int main(void)
{
    int err;

    printk("CW_1 Thermal Monitoring \n");

    if (!gpio_is_ready_dt(&led0)) {     //Check LED Ready
        printk("LED device not ready\n");
        return 0;
    }

    err = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);   //Config LED
    if (err < 0) {
        printk("LED configure failed (err=%d)\n", err);
        return 0;
    }

    err = bt_enable(NULL);  // Start bluetooth
    if (err) {
        printk("Bluetooth init failed (err=%d)\n", err);
        return 0;
    }

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);  //Start advertising
    if (err) {
        printk("Advertising failed to start (err=%d)\n", err);
        return 0;
    }
    ble_started = true;  //Allows BLE thread to advertise
   
    k_timer_init(&sample_timer, sample_timer_handler, NULL);
    k_timer_start(&sample_timer, K_MSEC(SAMPLE_PERIOD_MS), K_MSEC(SAMPLE_PERIOD_MS));
   
    while (1) {
        k_sleep(K_FOREVER); //Main can sleep as threads running
    }
}

