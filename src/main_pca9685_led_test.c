/*
 * =============================================================================
 * KLAVIATOR V4.0 - Stepper Motor + PCA9685 Solenoid Controller
 * =============================================================================
 *
 * SELECT MODE with the TEST_MODE define below:
 *
 *   TEST_MODE 1  ->  DM860E stepper motor test (runs on boot, loops forever)
 *                    Motor moves forward 1000 steps, pauses, reverses, repeats
 *                    PCA9685 / solenoids are NOT used
 *
 *   TEST_MODE 0  ->  PCA9685 solenoid test (runs on boot, full scale sequence)
 *                    Motor is NOT used
 *
 * Hardware connections:
 *
 *   MOTOR (DM860E):
 *     P1.09  ->  DM860E PUL+  (step pulses)
 *     P1.10  ->  DM860E DIR+  (direction)
 *     P1.11  ->  DM860E ENA+  (enable, active HIGH here)
 *     GND    ->  DM860E PUL-, DIR-, ENA-, GND
 *
 *   SOLENOIDS (PCA9685 via i2c21):
 *     P1.07  ->  PCA9685 SCL  (+ 4.7k pullup to 3.3V)
 *     P1.08  ->  PCA9685 SDA  (+ 4.7k pullup to 3.3V)
 *     3.3V   ->  PCA9685 VCC
 *     GND    ->  PCA9685 GND, OE, A0-A5
 *     PCA9685 CH0-15  ->  100R  ->  IRLZ44N gate
 *     IRLZ44N drain   ->  solenoid-  ->  12V
 *     1N4007 across each solenoid (cathode to 12V)
 *
 *   UART (optional, for dashboard when ENABLE_UART_MODE=1):
 *     P1.13  ->  USB-UART TX
 *     P1.14  ->  USB-UART RX
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>

/* =============================================================================
 * >>>  SET THIS TO CHOOSE WHAT RUNS ON BOOT  <<<
 *
 *   1 = DM860E stepper motor test
 *   0 = PCA9685 solenoid test
 * ============================================================================= */
#define TEST_MODE           1

/* =============================================================================
 * GENERAL CONFIGURATION
 * ============================================================================= */

#define SAFE_TEST_MODE      0   /* 0 = real hardware, 1 = print only */
#define ENABLE_UART_MODE    0   /* 0 = standalone, 1 = KDAA dashboard */

/* =============================================================================
 * MOTOR CONFIGURATION  (used when TEST_MODE=1)
 * ============================================================================= */

#define GPIO_DEV_NODE       DT_NODELABEL(gpio1)
#define PIN_STEP            9       /* P1.09  ->  DM860E PUL+ */
#define PIN_DIR             10      /* P1.10  ->  DM860E DIR+ */
#define PIN_ENA             11      /* P1.11  ->  DM860E ENA+ */

#define MOTOR_STEPS_PER_MOVE    1000    /* steps per direction */
#define MOTOR_STEP_DELAY_US     500     /* 500us per half-step = ~1000 steps/sec */
#define MOTOR_PAUSE_MS          1000    /* pause between directions */

#define DIR_FORWARD         1
#define DIR_REVERSE         0

static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_DEV_NODE);

/* =============================================================================
 * SOLENOID CONFIGURATION  (used when TEST_MODE=0)
 * ============================================================================= */

#define TOTAL_SOLENOIDS     16
#define BASE_MIDI_NOTE      60
#define KICK_DURATION_MS    20
#define MIN_PWM_PERCENT     18
#define TIMER_PERIOD_MS     5

/* i2c21: separate instance from UART20, free to use */
#define I2C_DEV_NODE            DT_NODELABEL(i2c21)
#define PCA9685_I2C_ADDR        0x40
#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_ALL_LED_ON_L    0xFA
#define PCA9685_ALL_LED_ON_H    0xFB
#define PCA9685_ALL_LED_OFF_L   0xFC
#define PCA9685_ALL_LED_OFF_H   0xFD
#define PCA9685_PRESCALE        0xFE
#define PCA9685_MODE1_SLEEP     0x10
#define PCA9685_MODE1_AI        0x20
#define PCA9685_MODE1_ALLCALL   0x01
#define PCA9685_MAX_PWM         4095
#define PCA9685_PRESCALE_VALUE  0x1D    /* ~203 Hz */

static const struct device *i2c_device = DEVICE_DT_GET(I2C_DEV_NODE);

/* =============================================================================
 * CLOCK
 * ============================================================================= */

static uint32_t system_time_offset = 0;
static inline uint32_t get_sync_time(void) { return k_uptime_get_32() - system_time_offset; }
void sync_clock(void) { system_time_offset = k_uptime_get_32(); printk("[SYNC] T=0\n"); }

/* =============================================================================
 * EVENT SYSTEM  (solenoid sequencer)
 * ============================================================================= */

typedef enum { EVENT_SYNC, EVENT_STOP, EVENT_MOVE, EVENT_STRIKE } event_type_t;

typedef struct {
    event_type_t type;
    uint32_t     target_time_ms;
    uint8_t      solenoid_number;
    uint8_t      velocity;
    uint16_t     duration_ms;
    uint16_t     sequence_number;
    uint8_t      hand;
} kdaa_event_t;

#define EVENT_BUFFER_SIZE 256
typedef struct {
    kdaa_event_t      events[EVENT_BUFFER_SIZE];
    volatile uint16_t write_index, read_index, count;
    uint32_t          total_added, total_executed, overflow_count;
} event_buffer_t;

static K_MUTEX_DEFINE(event_mutex);
static event_buffer_t event_buffer;

static void init_event_buffer(event_buffer_t *b) { memset(b, 0, sizeof(*b)); }

static bool add_event(event_buffer_t *b, const kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count >= EVENT_BUFFER_SIZE) {
        b->overflow_count++;
        k_mutex_unlock(&event_mutex);
        return false;
    }
    b->events[b->write_index] = *ev;
    b->write_index = (b->write_index + 1) & (EVENT_BUFFER_SIZE - 1);
    b->count++; b->total_added++;
    k_mutex_unlock(&event_mutex);
    return true;
}

static bool pop_event(event_buffer_t *b, kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count == 0) { k_mutex_unlock(&event_mutex); return false; }
    *ev = b->events[b->read_index];
    b->read_index = (b->read_index + 1) & (EVENT_BUFFER_SIZE - 1);
    b->count--; b->total_executed++;
    k_mutex_unlock(&event_mutex);
    return true;
}

static bool peek_event(const event_buffer_t *b, kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count == 0) { k_mutex_unlock(&event_mutex); return false; }
    *ev = b->events[b->read_index];
    k_mutex_unlock(&event_mutex);
    return true;
}

static void clear_events(event_buffer_t *b) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    b->write_index = b->read_index = b->count = 0;
    memset(b->events, 0, sizeof(b->events));
    k_mutex_unlock(&event_mutex);
}

/* =============================================================================
 * SOLENOID STATE
 * ============================================================================= */

struct Solenoid {
    uint8_t  velocity;
    bool     is_active;
    uint8_t  channel;
    bool     in_kick_phase;
    uint32_t kick_start_time;
    uint32_t duration_target_ms;
    uint32_t activation_time;
};
static struct Solenoid solenoids[TOTAL_SOLENOIDS];

/* =============================================================================
 * SEQUENCER THREAD
 * ============================================================================= */

#define SEQUENCER_STACK_SIZE 4096
#define SEQUENCER_PRIORITY   K_PRIO_COOP(1)
K_THREAD_STACK_DEFINE(sequencer_stack, SEQUENCER_STACK_SIZE);
static struct k_thread sequencer_thread;

/* =============================================================================
 * UART
 * ============================================================================= */

#if ENABLE_UART_MODE
#define MSG_SIZE 128
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char rx_buf[MSG_SIZE];
static int  rx_pos = 0;
#endif

/* =============================================================================
 * PCA9685 DRIVER
 * ============================================================================= */

static int pca9685_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write(i2c_device, buf, 2, PCA9685_I2C_ADDR);
    if (ret) printk("[ERROR] PCA9685 reg=0x%02X val=0x%02X err=%d\n", reg, val, ret);
    return ret;
}

static int pca9685_set_channel(uint8_t ch, uint16_t v) {
    if (ch >= 16) return -EINVAL;
    uint8_t reg = PCA9685_LED0_ON_L + (ch * 4);
    uint8_t buf[5];
    buf[0] = reg;
    if (v == 0)                    { buf[1]=0x00; buf[2]=0x00; buf[3]=0x00; buf[4]=0x10; }
    else if (v >= PCA9685_MAX_PWM) { buf[1]=0x00; buf[2]=0x10; buf[3]=0x00; buf[4]=0x00; }
    else                           { buf[1]=0x00; buf[2]=0x00;
                                     buf[3]=(uint8_t)(v & 0xFF); buf[4]=(uint8_t)(v >> 8); }
    int ret = i2c_write(i2c_device, buf, 5, PCA9685_I2C_ADDR);
    if (ret) printk("[ERROR] PCA9685 ch=%d val=%u err=%d\n", ch, v, ret);
    return ret;
}

static int pca9685_all_off(void) {
    int r = 0;
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_L,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_H,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10);
    return r;
}

/* =============================================================================
 * VELOCITY -> PWM  (integer quadratic, no FPU)
 * ============================================================================= */

static uint16_t velocity_to_pwm(uint8_t v) {
    if (v == 0)   return 0;
    if (v >= 127) return PCA9685_MAX_PWM;
    uint32_t v2  = (uint32_t)v * v;
    uint32_t pct = MIN_PWM_PERCENT + (v2 * (100U - MIN_PWM_PERCENT)) / (127U * 127U);
    return (uint16_t)((pct * PCA9685_MAX_PWM) / 100U);
}

static const char *note_name(uint8_t n) {
    static const char *names[] = {
        "C","C#","D","D#","E","F","F#","G","G#","A","A#","B","C'","C#'","D'","D#'"
    };
    return (n < TOTAL_SOLENOIDS) ? names[n] : "?";
}

/* =============================================================================
 * SOLENOID INIT
 * ============================================================================= */

static void init_solenoid_data(void) {
    printk("\n[INIT] Solenoid map (PCA9685 CH0-15):\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i] = (struct Solenoid){ .channel = (uint8_t)i };
        printk("  Sol %2d  CH%2d  Note %3d (%s)\n",
               i, i, BASE_MIDI_NOTE + i, note_name(i));
    }
    printk("\n");
}

static int init_pca9685(void) {
    printk("[INIT] PCA9685 on i2c21 addr=0x%02X prescaler=0x%02X (~200Hz)\n",
           PCA9685_I2C_ADDR, PCA9685_PRESCALE_VALUE);
    if (!device_is_ready(i2c_device)) {
        printk("[ERROR] i2c21 not ready. Check P1.07=SCL P1.08=SDA VCC=3.3V\n");
        return -ENODEV;
    }
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_SLEEP))                   return -EIO;
    k_usleep(500);
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VALUE))             return -EIO;
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) return -EIO;
    k_usleep(500);
    if (pca9685_write_reg(PCA9685_MODE2, 0x04))                                  return -EIO;
    if (pca9685_all_off())                                                        return -EIO;
    printk("[INIT] PCA9685 ready\n");
    return 0;
}

/* =============================================================================
 * SOLENOID CONTROL
 * ============================================================================= */

void activate_solenoid(uint8_t sol, uint8_t velocity, uint32_t duration_ms) {
    if (sol >= TOTAL_SOLENOIDS) return;
    struct Solenoid *s = &solenoids[sol];
    if (velocity > 0) {
        s->velocity = velocity;
        s->is_active = true;
        s->duration_target_ms = duration_ms;
        s->activation_time = s->kick_start_time = get_sync_time();
        s->in_kick_phase = true;
#if SAFE_TEST_MODE
        printk("[SAFE] ACTIVATE sol=%2d vel=%3d dur=%4dms T=%u\n",
               sol, velocity, duration_ms, s->activation_time);
#else
        pca9685_set_channel(s->channel, PCA9685_MAX_PWM);
        printk("[KICK]    sol=%2d ch=%2d vel=%3d dur=%4dms T=%u\n",
               sol, s->channel, velocity, duration_ms, s->activation_time);
#endif
    } else {
        s->velocity = 0;
        s->is_active = false;
        s->in_kick_phase = false;
        s->duration_target_ms = 0;
#if SAFE_TEST_MODE
        printk("[SAFE] RELEASE sol=%2d T=%u\n", sol, get_sync_time());
#else
        pca9685_set_channel(s->channel, 0);
        printk("[RELEASE] sol=%2d ch=%2d T=%u\n",
               sol, s->channel, get_sync_time());
#endif
    }
}

void deactivate_solenoid(uint8_t sol) { activate_solenoid(sol, 0, 0); }

void deactivate_all_solenoids(void) {
    printk("[ALL-OFF]\n");
#if !SAFE_TEST_MODE
    pca9685_all_off();
#endif
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
        solenoids[i].in_kick_phase = false;
        solenoids[i].duration_target_ms = 0;
    }
}

/* =============================================================================
 * CONTROL TIMER  (kick->hold + auto-release)
 * ============================================================================= */

static void control_timer_cb(struct k_timer *timer) {
    uint32_t now = get_sync_time();
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        struct Solenoid *s = &solenoids[i];
        if (!s->is_active) continue;

        if (s->in_kick_phase &&
            (now - s->kick_start_time) >= KICK_DURATION_MS) {
            s->in_kick_phase = false;
            uint16_t hold = velocity_to_pwm(s->velocity);
#if !SAFE_TEST_MODE
            pca9685_set_channel(s->channel, hold);
#endif
            printk("[HOLD]    sol=%2d ch=%2d vel=%3d pwm=%4u T=%u%s\n",
                   i, s->channel, s->velocity, hold, now,
                   SAFE_TEST_MODE ? " (SIM)" : "");
        }

        if (s->duration_target_ms > 0 &&
            (now - s->activation_time) >= s->duration_target_ms) {
            printk("[AUTO-REL] sol=%2d dur=%4dms T=%u%s\n",
                   i, s->duration_target_ms, now,
                   SAFE_TEST_MODE ? " (SIM)" : "");
            deactivate_solenoid(i);
        }
    }
}

K_TIMER_DEFINE(control_timer, control_timer_cb, NULL);

/* =============================================================================
 * SEQUENCER THREAD
 * ============================================================================= */

static void sequencer_loop(void *a1, void *a2, void *a3) {
    kdaa_event_t ev;
    printk("[SEQ] Started prio=%d poll=0.5ms\n", SEQUENCER_PRIORITY);
    while (1) {
        uint32_t now = get_sync_time();
        while (peek_event(&event_buffer, &ev)) {
            if (now < ev.target_time_ms) break;
            pop_event(&event_buffer, &ev);
            switch (ev.type) {
            case EVENT_STRIKE:
                if (ev.solenoid_number < TOTAL_SOLENOIDS) {
                    activate_solenoid(ev.solenoid_number,
                                      ev.velocity, ev.duration_ms);
                    printk("[EXEC] #%u STRIKE sol=%2d vel=%3d dur=%4dms"
                           " T=%u actual=%u delta=%d\n",
                           ev.sequence_number, ev.solenoid_number,
                           ev.velocity, ev.duration_ms,
                           ev.target_time_ms, now,
                           (int)(now - ev.target_time_ms));
                }
                break;
            case EVENT_STOP:
                printk("[EXEC] #%u STOP\n", ev.sequence_number);
                clear_events(&event_buffer);
                deactivate_all_solenoids();
                break;
            case EVENT_SYNC:
                printk("[EXEC] #%u SYNC\n", ev.sequence_number);
                sync_clock();
                break;
            case EVENT_MOVE:
                printk("[EXEC] #%u MOVE hand=%d pos=%d\n",
                       ev.sequence_number, ev.hand, ev.solenoid_number);
                break;
            default:
                printk("[ERROR] Unknown event %d\n", ev.type);
                break;
            }
        }
        k_usleep(500);
    }
}

/* =============================================================================
 * UART  (compiled out when ENABLE_UART_MODE=0)
 * ============================================================================= */

#if ENABLE_UART_MODE
static void uart_cb(const struct device *dev, void *user_data) {
    uint8_t c;
    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev)) return;
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            if (rx_pos > 0) {
                rx_buf[rx_pos] = '\0';
                k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
                rx_pos = 0;
            }
        } else if (rx_pos < (MSG_SIZE - 1)) {
            rx_buf[rx_pos++] = c;
        } else {
            rx_pos = 0;
        }
    }
}

static void parse_kdaa(char *msg) {
    int seq, hand, note, vel, t_ms, dur;
    if (!strcasecmp(msg, "sync"))  { sync_clock();               return; }
    if (!strcasecmp(msg, "all:0")) { deactivate_all_solenoids(); return; }
    if (sscanf(msg, "%d:%d:%d:%d:%d:%d",
               &seq, &hand, &note, &vel, &t_ms, &dur) == 6) {
        int sol = note - BASE_MIDI_NOTE;
        if (sol >= 0 && sol < TOTAL_SOLENOIDS && vel >= 0 && vel <= 127)
            activate_solenoid((uint8_t)sol, (uint8_t)vel, (uint32_t)dur);
        else printk("[ERR] Invalid note/velocity\n");
        return;
    }
    if (sscanf(msg, "%d:%d", &note, &vel) == 2) {
        int sol = note - BASE_MIDI_NOTE;
        if (sol >= 0 && sol < TOTAL_SOLENOIDS && vel >= 0 && vel <= 127)
            activate_solenoid((uint8_t)sol, (uint8_t)vel, 0);
        else printk("[ERR] Invalid note/velocity\n");
        return;
    }
    printk("[ERR] Use S:H:N:V:T:D or NOTE:VEL\n");
}
#endif

/* =============================================================================
 * MOTOR TEST  (TEST_MODE=1)
 *
 * Sends step pulses to DM860E.
 * Adjust MOTOR_STEP_DELAY_US to change speed:
 *   500us  = ~1000 steps/sec  (medium)
 *   200us  = ~2500 steps/sec  (fast)
 *   1000us = ~500  steps/sec  (slow)
 *
 * Adjust MOTOR_STEPS_PER_MOVE to change distance.
 * At DM860E default 200 steps/rev: 1000 steps = 5 full rotations.
 * ============================================================================= */

#if TEST_MODE

static int init_motor(void) {
    printk("[MOTOR] Initialising DM860E on gpio1\n");
    printk("[MOTOR] P1.09=STEP  P1.10=DIR  P1.11=ENA\n");

    if (!device_is_ready(gpio_dev)) {
        printk("[ERROR] gpio1 not ready\n");
        return -ENODEV;
    }

    gpio_pin_configure(gpio_dev, PIN_STEP, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_DIR,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_ENA,  GPIO_OUTPUT_INACTIVE);

    /* Enable motor driver */
    gpio_pin_set(gpio_dev, PIN_ENA, 1);
    k_msleep(100); /* DM860E needs time to enable */

    printk("[MOTOR] DM860E enabled\n");
    return 0;
}

static void step_motor(uint32_t steps, int direction, uint32_t delay_us)
{
    gpio_pin_set(gpio_dev, PIN_DIR, direction);
    k_usleep(10); /* direction settle time */

    for (uint32_t i = 0; i < steps; i++) {
        gpio_pin_set(gpio_dev, PIN_STEP, 1);
        k_usleep(delay_us);
        gpio_pin_set(gpio_dev, PIN_STEP, 0);
        k_usleep(delay_us);
    }
}

static void run_motor_test(void) {
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  DM860E MOTOR TEST  -  FORWARD / REVERSE LOOP       ║\n");
    printk("╚══════════════════════════════════════════════════════╝\n");
    printk("[MOTOR] Steps per move : %d\n",   MOTOR_STEPS_PER_MOVE);
    printk("[MOTOR] Step delay     : %d us\n", MOTOR_STEP_DELAY_US);
    printk("[MOTOR] Speed          : ~%d steps/sec\n",
           1000000 / (MOTOR_STEP_DELAY_US * 2));
    printk("[MOTOR] Pause between  : %d ms\n\n", MOTOR_PAUSE_MS);

    uint32_t pass = 0;
    while (1) {
        printk("[MOTOR] Pass %u  FORWARD  %d steps...\n",
               pass, MOTOR_STEPS_PER_MOVE);
        step_motor(MOTOR_STEPS_PER_MOVE, DIR_FORWARD, MOTOR_STEP_DELAY_US);

        printk("[MOTOR] Pause %dms\n", MOTOR_PAUSE_MS);
        k_msleep(MOTOR_PAUSE_MS);

        printk("[MOTOR] Pass %u  REVERSE  %d steps...\n",
               pass, MOTOR_STEPS_PER_MOVE);
        step_motor(MOTOR_STEPS_PER_MOVE, DIR_REVERSE, MOTOR_STEP_DELAY_US);

        printk("[MOTOR] Pause %dms\n\n", MOTOR_PAUSE_MS);
        k_msleep(MOTOR_PAUSE_MS);

        pass++;
    }
}

#endif /* TEST_MODE */

/* =============================================================================
 * SOLENOID TEST SEQUENCE  (TEST_MODE=0)
 * ============================================================================= */

#if !TEST_MODE

static void queue_strike(uint16_t seq, uint32_t t,
                         uint8_t sol, uint8_t vel, uint16_t dur) {
    kdaa_event_t ev = {
        .type            = EVENT_STRIKE,
        .target_time_ms  = t,
        .solenoid_number = sol,
        .velocity        = vel,
        .duration_ms     = dur,
        .sequence_number = seq,
        .hand            = 0,
    };
    if (!add_event(&event_buffer, &ev))
        printk("[ERR] Buffer full seq=%u\n", seq);
}

static void run_solenoid_test(void) {
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  SOLENOID TEST  -  AUTONOMOUS BOOT PLAY             ║\n");
    printk("╚══════════════════════════════════════════════════════╝\n");
    printk("[TEST] Phase 1  T=0.5-2.0s  : vel sweep sol0 (30/64/127)\n");
    printk("[TEST] Phase 2  T=3.0s      : 500ms hold, watch KICK->HOLD\n");
    printk("[TEST] Phase 3  T=4.0s      : rapid fire x4\n");
    printk("[TEST] Phase 4  T=5.5s      : all 16 ascending\n");
    printk("[TEST] Phase 5  T=9.0s      : second pass softer\n");
    printk("[TEST] Phase 6  T=12.5s     : STOP\n\n");

    uint16_t seq = 1;

    queue_strike(seq++,  500, 0,  30,  80);
    queue_strike(seq++, 1200, 0,  64,  80);
    queue_strike(seq++, 2000, 0, 127,  80);
    queue_strike(seq++, 3000, 0,  90, 500);
    queue_strike(seq++, 4000, 0, 100,  60);
    queue_strike(seq++, 4080, 0, 100,  60);
    queue_strike(seq++, 4160, 0, 100,  60);
    queue_strike(seq++, 4240, 0, 100,  60);

    for (int i = 0; i < TOTAL_SOLENOIDS; i++)
        queue_strike(seq++, (uint32_t)(5500 + i * 200), (uint8_t)i, 90, 120);

    for (int i = 0; i < TOTAL_SOLENOIDS; i++)
        queue_strike(seq++, (uint32_t)(9000 + i * 200), (uint8_t)i, 70, 100);

    kdaa_event_t stop = {
        .type           = EVENT_STOP,
        .target_time_ms = 12500,
        .sequence_number = seq++,
        .hand           = 0,
    };
    add_event(&event_buffer, &stop);

    printk("[TEST] %u events queued. Starts in ~500ms.\n\n", seq - 1);
}

#endif /* !TEST_MODE */

/* =============================================================================
 * MAIN
 * ============================================================================= */

int main(void)
{
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  KLAVIATOR V4.0                                     ║\n");

#if TEST_MODE
    printk("║  MODE: DM860E MOTOR TEST                            ║\n");
#else
    printk("║  MODE: PCA9685 SOLENOID TEST                        ║\n");
#endif

    printk("╚══════════════════════════════════════════════════════╝\n\n");

    k_msleep(100);

    /* ------------------------------------------------------------------ */
#if TEST_MODE
    /* ---- MOTOR TEST PATH ---- */

    if (init_motor() < 0) {
        printk("[ERROR] Motor init failed - check wiring\n");
        printk("[ERROR] P1.09=STEP P1.10=DIR P1.11=ENA GND=GND\n");
        while (1) { k_msleep(1000); }
    }

    printk("[READY] Motor ready. Starting test loop.\n\n");
    run_motor_test(); /* loops forever */

    /* ------------------------------------------------------------------ */
#else
    /* ---- SOLENOID TEST PATH ---- */

    if (init_pca9685() < 0)
        printk("[WARN] PCA9685 failed - check wiring\n");

    init_solenoid_data();
    sync_clock();

    k_timer_start(&control_timer,
                  K_MSEC(TIMER_PERIOD_MS), K_MSEC(TIMER_PERIOD_MS));
    printk("[INIT] Control timer %dms kick=%dms min_hold=%d%%\n",
           TIMER_PERIOD_MS, KICK_DURATION_MS, MIN_PWM_PERCENT);

    init_event_buffer(&event_buffer);
    printk("[INIT] Event buffer %d slots\n", EVENT_BUFFER_SIZE);

    k_thread_create(&sequencer_thread, sequencer_stack,
                    K_THREAD_STACK_SIZEOF(sequencer_stack),
                    sequencer_loop, NULL, NULL, NULL,
                    SEQUENCER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sequencer_thread, "sequencer");
    printk("[INIT] Sequencer started\n");

#if SAFE_TEST_MODE
    printk("[WARN] SAFE_TEST_MODE=1 - no hardware output\n");
#endif
    printk("[READY] System ready\n\n");

    run_solenoid_test();

#if ENABLE_UART_MODE
    printk("UART KDAA V4.0. Full: S:H:N:V:T:D  Simple: NOTE:VEL\n\n");
    if (!device_is_ready(uart_dev)) {
        printk("[WARN] UART not ready\n");
        uint32_t n = 0;
        while (1) { printk("[HB] %u\n", n++); k_msleep(1000); }
    }
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    char msg[MSG_SIZE];
    uint32_t hb = 0;
    while (1) {
        if (k_msgq_get(&uart_msgq, msg, K_MSEC(1000)) == 0)
            parse_kdaa(msg);
        else
            printk("[HB] waiting (%u)\n", hb++);
    }
#else
    printk("[INFO] Standalone. Set ENABLE_UART_MODE=1 for dashboard.\n\n");
    uint32_t n = 0;
    while (1) { printk("[HB] alive %u\n", n++); k_msleep(1000); }
#endif

#endif /* TEST_MODE / solenoid */

    return 0;
}