/*
 * =============================================================================
 * KLAVIATOR V4.0 - Stepper Motor + PCA9685 Solenoid Controller
 * =============================================================================
 *
 * MOTOR_TEST_MODE 1  ->  Shuttle test: motor beveger seg automatisk frem/tilbake
 * MOTOR_TEST_MODE 0  ->  KDAA modus: motor og solenoider styres fra dashboard
 *
 * Hardware connections:
 *
 *   MOTOR (DM860E via lineaer aktuator):
 *     P1.09  ->  DM860E PUL+  (step pulses)
 *     P1.10  ->  DM860E DIR+  (direction)
 *     P1.11  ->  DM860E ENA+  (enable, active HIGH)
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
 *   UART (for dashboard):
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
#include <strings.h>

/* =============================================================================
 * MOTOR TEST MODE
 * 1 = Shuttle test (automatisk frem/tilbake)
 * 0 = KDAA modus (dashboard-styrt)
 * ============================================================================= */
#define MOTOR_TEST_MODE     0

/* =============================================================================
 * GENERAL CONFIGURATION
 * ============================================================================= */

#define SAFE_TEST_MODE      0   /* 0 = real hardware, 1 = print only */
#define ENABLE_UART_MODE    1   /* 0 = standalone, 1 = KDAA dashboard */

static void queue_strike(uint16_t seq, uint32_t t, uint8_t sol, uint8_t vel, uint16_t dur);

/* =============================================================================
 * MOTOR CONFIGURATION
 * ============================================================================= */

#define GPIO_DEV_NODE           DT_NODELABEL(gpio1)
#define PIN_STEP                9
#define PIN_DIR                 10
#define PIN_ENA                 11

#define SHUTTLE_POS_A           0
#define SHUTTLE_POS_B           300
#define SHUTTLE_PAUSE_MS        500

#define STEPS_PER_MM            40
#define RAIL_MIN_MM             0
#define RAIL_MAX_MM             590

#define MOTOR_SPEED_NORMAL_US   500
#define MOTOR_SPEED_SLOW_US     1500
#define MOTOR_SPEED_HOMING_US   2000
#define MOTOR_ACCEL_STEPS       200
#define HOMING_MAX_STEPS        2000

#define DIR_FORWARD             1
#define DIR_REVERSE             0

static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_DEV_NODE);

/* =============================================================================
 * SOLENOID CONFIGURATION
 * ============================================================================= */

#define TOTAL_SOLENOIDS     16
#define BASE_MIDI_NOTE      36   /* C2 - startposisjon for solenoid 0 (CH0) */

/* Mapping: MIDI-note -> solenoid-indeks
 * Hvite tangenter (CH0-7):  C2(36)=0, D2(38)=1, E2(40)=2, F2(41)=3,
 *                           G2(43)=4, A2(45)=5, B2(47)=6, C3(48)=7
 * Svarte tangenter (CH8-15): C#2(37)=8, D#2(39)=9, F#2(42)=10, G#2(44)=11,
 *                            A#2(46)=12, C#3(49)=13, D#3(51)=14, F#3(54)=15 */
static const int8_t midi_to_sol[20] = {
     0,  /* 36 C2  -> sol 0  (hvit) */
     8,  /* 37 C#2 -> sol 8  (svart) */
     1,  /* 38 D2  -> sol 1  (hvit) */
     9,  /* 39 D#2 -> sol 9  (svart) */
     2,  /* 40 E2  -> sol 2  (hvit) */
     3,  /* 41 F2  -> sol 3  (hvit) */
    10,  /* 42 F#2 -> sol 10 (svart) */
     4,  /* 43 G2  -> sol 4  (hvit) */
    11,  /* 44 G#2 -> sol 11 (svart) */
     5,  /* 45 A2  -> sol 5  (hvit) */
    12,  /* 46 A#2 -> sol 12 (svart) */
     6,  /* 47 B2  -> sol 6  (hvit) */
     7,  /* 48 C3  -> sol 7  (hvit) */
    13,  /* 49 C#3 -> sol 13 (svart) */
    -1,  /* 50 D3  -> ingen */
    14,  /* 51 D#3 -> sol 14 (svart) */
    -1,  /* 52 E3  -> ingen */
    -1,  /* 53 F3  -> ingen */
    15,  /* 54 F#3 -> sol 15 (svart) */
    -1,  /* 55 G3  -> ingen */
};

static inline int8_t note_to_solenoid(uint8_t note) {
    if (note < BASE_MIDI_NOTE || note > 55) return -1;
    return midi_to_sol[note - BASE_MIDI_NOTE];
}

#define KICK_DURATION_MIN_MS  8
#define KICK_DURATION_MAX_MS  28

static inline uint32_t kick_duration_for_vel(uint8_t vel) {
    return KICK_DURATION_MIN_MS +
           ((uint32_t)vel * (KICK_DURATION_MAX_MS - KICK_DURATION_MIN_MS)) / 127U;
}

#define MIN_PWM_PERCENT     18
#define TIMER_PERIOD_MS     2

/* =============================================================================
 * PCA9685 CONFIGURATION
 * ============================================================================= */

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
#define PCA9685_PRESCALE_VALUE  0x1D

static const struct device *i2c_device = DEVICE_DT_GET(I2C_DEV_NODE);

/* =============================================================================
 * CLOCK
 * ============================================================================= */

static uint32_t system_time_offset = 0;
static inline uint32_t get_sync_time(void) { return k_uptime_get_32() - system_time_offset; }
void sync_clock(void) { system_time_offset = k_uptime_get_32(); printk("[SYNC] T=0\n"); }

/* =============================================================================
 * EVENT SYSTEM
 * ============================================================================= */

typedef enum { EVENT_SYNC, EVENT_STOP, EVENT_MOVE, EVENT_STRIKE } event_type_t;

typedef struct {
    event_type_t type;
    uint32_t     target_time_ms;
    uint8_t      note_number;
    uint8_t      velocity;
    uint16_t     duration_ms;
    uint16_t     sequence_number;
    uint8_t      hand;
} kdaa_event_t;

#define EVENT_BUFFER_SIZE 2048
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
        printk("ERROR:BUFFER_FULL\n");
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
    uint32_t kick_duration_ms;
    uint32_t duration_target_ms;
    uint32_t activation_time;
};
static struct Solenoid solenoids[TOTAL_SOLENOIDS];

/* =============================================================================
 * THREAD DEFINITIONS
 * ============================================================================= */

#define SEQUENCER_STACK_SIZE 4096
#define SEQUENCER_PRIORITY   K_PRIO_COOP(1)
K_THREAD_STACK_DEFINE(sequencer_stack, SEQUENCER_STACK_SIZE);
static struct k_thread sequencer_thread;

#define MOTOR_STACK_SIZE    4096
#define MOTOR_PRIORITY      K_PRIO_COOP(2)
K_THREAD_STACK_DEFINE(motor_stack, MOTOR_STACK_SIZE);
static struct k_thread motor_thread;

static struct {
    int32_t  position_steps;
    int32_t  position_mm;
    bool     is_homed;
    bool     is_moving;
    bool     shuttle_running;
} motor_state = { 0, 0, false, false, false };

typedef struct {
    int32_t  target_mm;
    uint32_t speed_us;
} motor_cmd_t;

K_MSGQ_DEFINE(motor_msgq, sizeof(motor_cmd_t), 8, 4);

/* =============================================================================
 * UART
 * ============================================================================= */

#if ENABLE_UART_MODE
#define MSG_SIZE 128
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 1024, 4);
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
    return i2c_write(i2c_device, buf, 2, PCA9685_I2C_ADDR);
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
    return i2c_write(i2c_device, buf, 5, PCA9685_I2C_ADDR);
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
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_SLEEP))                      return -EIO;
    k_usleep(500);
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VALUE))                return -EIO;
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) return -EIO;
    k_usleep(500);
    if (pca9685_write_reg(PCA9685_MODE2, 0x04))                                     return -EIO;
    if (pca9685_all_off())                                                           return -EIO;
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
        s->kick_duration_ms = kick_duration_for_vel(velocity);
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
        printk("[RELEASE] sol=%2d ch=%2d T=%u\n", sol, s->channel, get_sync_time());
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
 * k_work brukes fordi I2C IKKE kan kalles fra ISR-kontekst
 * ============================================================================= */

static struct k_work solenoid_work;

static void solenoid_work_handler(struct k_work *work) {
    uint32_t now = get_sync_time();
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        struct Solenoid *s = &solenoids[i];
        if (!s->is_active) continue;

        if (s->in_kick_phase &&
            (now - s->kick_start_time) >= s->kick_duration_ms) {
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

static void control_timer_cb(struct k_timer *timer) {
    k_work_submit(&solenoid_work);
}

K_TIMER_DEFINE(control_timer, control_timer_cb, NULL);

/* =============================================================================
 * MOTOR DRIVER  (DM860E stepper via linear aktuator)
 * ============================================================================= */

static void motor_set_dir(int dir) {
    gpio_pin_set(gpio_dev, PIN_DIR, dir);
    k_usleep(5);
}

static void motor_step_pulse(uint32_t delay_us) {
    gpio_pin_set(gpio_dev, PIN_STEP, 1);
    k_usleep(delay_us);
    gpio_pin_set(gpio_dev, PIN_STEP, 0);
    k_usleep(delay_us);
}

static void motor_run_steps(int dir, uint32_t steps, uint32_t speed_us) {
    if (steps == 0) return;
    motor_set_dir(dir);
    motor_state.is_moving = true;
    for (uint32_t i = 0; i < steps; i++) {
        uint32_t us = speed_us;
        if (steps > (uint32_t)(MOTOR_ACCEL_STEPS * 2)) {
            if (i < MOTOR_ACCEL_STEPS) {
                uint32_t f = MOTOR_ACCEL_STEPS - i;
                us = speed_us + (f * speed_us * 2) / MOTOR_ACCEL_STEPS;
            } else if (i > steps - MOTOR_ACCEL_STEPS) {
                uint32_t f = i - (steps - MOTOR_ACCEL_STEPS);
                us = speed_us + (f * speed_us * 2) / MOTOR_ACCEL_STEPS;
            }
        }
        motor_step_pulse(us);
        if (dir == DIR_FORWARD) motor_state.position_steps++;
        else                    motor_state.position_steps--;
    }
    motor_state.position_mm = motor_state.position_steps / STEPS_PER_MM;
    motor_state.is_moving = false;
}

static void motor_home(void) {
    printk("[MOTOR] Homing...\n");
    gpio_pin_set(gpio_dev, PIN_ENA, 1);
    k_msleep(50);
    motor_set_dir(DIR_REVERSE);
    for (uint32_t i = 0; i < HOMING_MAX_STEPS; i++) {
        motor_step_pulse(MOTOR_SPEED_HOMING_US);
    }
    k_msleep(200);
    motor_run_steps(DIR_FORWARD, 5 * STEPS_PER_MM, MOTOR_SPEED_HOMING_US);
    motor_state.position_steps = 0;
    motor_state.position_mm = 0;
    motor_state.is_homed = true;
    printk("[MOTOR] Homed. Posisjon = 0mm\n");
}

static void motor_goto_mm(int32_t target_mm, uint32_t speed_us) {
    if (target_mm < RAIL_MIN_MM) target_mm = RAIL_MIN_MM;
    if (target_mm > RAIL_MAX_MM) target_mm = RAIL_MAX_MM;
    int32_t  target_steps = (int32_t)(target_mm * STEPS_PER_MM);
    int32_t  delta        = target_steps - motor_state.position_steps;
    if (delta == 0) return;
    int      dir   = (delta > 0) ? DIR_FORWARD : DIR_REVERSE;
    uint32_t steps = (uint32_t)abs(delta);
    printk("[MOTOR] %dmm -> %dmm (%d steg)\n",
           motor_state.position_mm, target_mm, steps);
    motor_run_steps(dir, steps, speed_us);
    printk("[MOTOR] Ankom %dmm\n", motor_state.position_mm);
}

static void motor_queue_move(int32_t target_mm, uint32_t speed_us) {
    motor_cmd_t cmd = { .target_mm = target_mm, .speed_us = speed_us };
    if (k_msgq_put(&motor_msgq, &cmd, K_NO_WAIT) != 0) {
        printk("[WARN] Motor ko full\n");
    }
}

static void motor_thread_fn(void *a1, void *a2, void *a3) {
    printk("[MOTOR] Trad startet\n");
    if (!device_is_ready(gpio_dev)) {
        printk("[ERROR] Motor GPIO ikke klar\n");
        return;
    }
    gpio_pin_configure(gpio_dev, PIN_STEP, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_DIR,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_ENA,  GPIO_OUTPUT_INACTIVE);

    motor_home();

#if MOTOR_TEST_MODE
    motor_state.shuttle_running = true;
    bool at_a = true;
    uint32_t pass = 0;
    printk("[MOTOR] Shuttle-modus: %dmm <-> %dmm\n", SHUTTLE_POS_A, SHUTTLE_POS_B);
    while (1) {
        if (at_a) {
            printk("[MOTOR] Pass %u -> %dmm\n", pass, SHUTTLE_POS_B);
            motor_goto_mm(SHUTTLE_POS_B, MOTOR_SPEED_NORMAL_US);
            at_a = false;
        } else {
            printk("[MOTOR] Pass %u -> %dmm\n", pass, SHUTTLE_POS_A);
            motor_goto_mm(SHUTTLE_POS_A, MOTOR_SPEED_NORMAL_US);
            at_a = true;
            pass++;
        }
        k_msleep(SHUTTLE_PAUSE_MS);
    }
#else
    motor_cmd_t cmd;
    while (1) {
        if (k_msgq_get(&motor_msgq, &cmd, K_FOREVER) == 0) {
            printk("[MOTOR] CMD -> %dmm\n", cmd.target_mm);
            motor_goto_mm(cmd.target_mm, cmd.speed_us);
        }
    }
#endif
}

/* =============================================================================
 * SEQUENCER THREAD
 * ============================================================================= */

static void sequencer_loop(void *a1, void *a2, void *a3) {
    kdaa_event_t ev;
    while (1) {
        uint32_t now = get_sync_time();
        while (peek_event(&event_buffer, &ev)) {
            if (now < ev.target_time_ms) break;
            pop_event(&event_buffer, &ev);
            switch (ev.type) {
            case EVENT_MOVE:
                motor_queue_move((int32_t)ev.note_number, MOTOR_SPEED_NORMAL_US);
                break;
            case EVENT_STRIKE: {
                int8_t sol_idx = note_to_solenoid(ev.note_number);
                if (sol_idx >= 0) {
                    activate_solenoid((uint8_t)sol_idx, ev.velocity, ev.duration_ms);
                }
                if (ev.velocity > 0) {
                    printk("HIT:%d\n", ev.note_number);
                }
                break;
            }
            case EVENT_STOP:
                clear_events(&event_buffer);
                deactivate_all_solenoids();
                break;
            case EVENT_SYNC:
                sync_clock();
                break;
            default:
                break;
            }
        }
        k_usleep(50);
    }
}

/* =============================================================================
 * UART
 * ============================================================================= */

#if ENABLE_UART_MODE
static void uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf[64];
    int len;
    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev)) return;
    while ((len = uart_fifo_read(uart_dev, buf, sizeof(buf))) > 0) {
        for (int i = 0; i < len; i++) {
            uint8_t c = buf[i];
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
}

static void parse_kdaa(char *msg) {
    int seq, note, vel, t_ms, dur;
    char hand;

    if (strncmp(msg, "SYNC", 4) == 0) {
        clear_events(&event_buffer);
        sync_clock();
        printk("[MCU] SYNC OK - Clock Reset\n");
        return;
    }

    if (strcmp(msg, "STOP") == 0 || strcasecmp(msg, "all:0") == 0) {
        deactivate_all_solenoids();
        init_event_buffer(&event_buffer);
        printk("[MCU] HALTED\n");
        return;
    }

    if (msg[0] == 'M' && msg[1] == ':') {
        int p; uint32_t t;
        if (sscanf(msg, "M:%c:%d:%u", &hand, &p, &t) == 3) {
            kdaa_event_t ev = {
                .type = EVENT_MOVE,
                .target_time_ms = t,
                .hand = hand,
                .note_number = (uint8_t)p,
                .sequence_number = 0
            };
            add_event(&event_buffer, &ev);
            return;
        }
    }

    if (msg[0] == 'S' && msg[1] == ':') {
        if (sscanf(msg, "S:%c:%d:%d:%d:%d", &hand, &note, &vel, &t_ms, &dur) == 5) {
            if (note >= 0 && note <= 127 && vel >= 0 && vel <= 127) {
                kdaa_event_t ev = {
                    .type = EVENT_STRIKE,
                    .target_time_ms = t_ms,
                    .note_number = (uint8_t)note,
                    .velocity = (uint8_t)vel,
                    .duration_ms = (uint16_t)dur,
                    .hand = hand
                };
                add_event(&event_buffer, &ev);
            } else {
                printk("[ERR] Invalid note/velocity\n");
            }
            return;
        }
        if (sscanf(msg, "S:%c:%d:%d:%d", &hand, &note, &vel, &t_ms) == 4) {
            if (note >= 0 && note <= 127 && vel >= 0 && vel <= 127) {
                kdaa_event_t ev = {
                    .type = EVENT_STRIKE,
                    .target_time_ms = t_ms,
                    .note_number = (uint8_t)note,
                    .velocity = (uint8_t)vel,
                    .duration_ms = 50,
                    .hand = hand
                };
                add_event(&event_buffer, &ev);
            } else {
                printk("[ERR] Invalid note/velocity\n");
            }
            return;
        }
    }

    int hand_int;
    if (sscanf(msg, "%d:%d:%d:%d:%d:%d", &seq, &hand_int, &note, &vel, &t_ms, &dur) == 6) {
        hand = (char)hand_int;
        if (note >= 0 && note <= 127 && vel >= 0 && vel <= 127)
            queue_strike(seq, t_ms, (uint8_t)note, (uint8_t)vel, (uint16_t)dur);
        else printk("[ERR] Invalid note/velocity\n");
        return;
    }

    if (sscanf(msg, "%d:%d", &note, &vel) == 2) {
        if (note >= 0 && note <= 127 && vel >= 0 && vel <= 127) {
            int8_t sol_idx = note_to_solenoid((uint8_t)note);
            if (sol_idx >= 0) activate_solenoid((uint8_t)sol_idx, (uint8_t)vel, 0);
        } else printk("[ERR] Invalid note/velocity\n");
        return;
    }
    printk("[ERR] Use S:H:N:V:T:D or M:H:P:T or NOTE:VEL\n");
}
#endif

/* =============================================================================
 * QUEUE STRIKE HELPER
 * ============================================================================= */

static void queue_strike(uint16_t seq, uint32_t t, uint8_t note, uint8_t vel, uint16_t dur) {
    kdaa_event_t ev = {
        .type            = EVENT_STRIKE,
        .target_time_ms  = t,
        .note_number     = note,
        .velocity        = vel,
        .duration_ms     = dur,
        .sequence_number = seq,
        .hand            = 0,
    };
    if (!add_event(&event_buffer, &ev))
        printk("[ERR] Buffer full seq=%u\n", seq);
}

/* =============================================================================
 * MAIN
 * ============================================================================= */

int main(void)
{
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  KLAVIATOR V4.0                                     ║\n");
#if MOTOR_TEST_MODE
    printk("║  MODE: MOTOR SHUTTLE TEST                           ║\n");
#else
    printk("║  MODE: KLAVIATOR MIDI SEQUENCER + MOTOR             ║\n");
#endif
    printk("╚══════════════════════════════════════════════════════╝\n\n");

    k_msleep(100);

    if (init_pca9685() < 0)
        printk("[WARN] PCA9685 failed - check wiring\n");

    init_solenoid_data();
    sync_clock();

    k_timer_start(&control_timer, K_MSEC(TIMER_PERIOD_MS), K_MSEC(TIMER_PERIOD_MS));
    printk("[INIT] Control timer %dms kick=%d-%dms (vel-adaptive) min_hold=%d%%\n",
           TIMER_PERIOD_MS, KICK_DURATION_MIN_MS, KICK_DURATION_MAX_MS, MIN_PWM_PERCENT);

    k_work_init(&solenoid_work, solenoid_work_handler);
    init_event_buffer(&event_buffer);
    printk("[INIT] Event buffer %d slots\n", EVENT_BUFFER_SIZE);

    k_thread_create(&sequencer_thread, sequencer_stack,
                    K_THREAD_STACK_SIZEOF(sequencer_stack),
                    sequencer_loop, NULL, NULL, NULL,
                    SEQUENCER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sequencer_thread, "sequencer");
    printk("[INIT] Sequencer startet\n");

    k_thread_create(&motor_thread, motor_stack,
                    K_THREAD_STACK_SIZEOF(motor_stack),
                    motor_thread_fn, NULL, NULL, NULL,
                    MOTOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&motor_thread, "motor");
    printk("[INIT] Motor-trad startet\n");

#if SAFE_TEST_MODE
    printk("[WARN] SAFE_TEST_MODE=1 - no hardware output\n");
#endif
    printk("[READY] System ready\n\n");

#if ENABLE_UART_MODE
    printk("UART KDAA V4.0. Full: S:H:N:V:T:D  Move: M:H:P:T\n\n");
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
            printk("[HB] waiting (%u) motor=%dmm\n", hb++, motor_state.position_mm);
    }
#else
    printk("[INFO] Standalone. Set ENABLE_UART_MODE=1 for dashboard.\n\n");
    uint32_t n = 0;
    while (1) {
        printk("[HB] alive %u motor=%dmm\n", n++, motor_state.position_mm);
        k_msleep(1000);
    }
#endif

    return 0;
}
