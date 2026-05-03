/*
 * KLAVIATOR V4.0 
 
 *   Set to 1 = SHUTTLE TEST(moving motor)
      
 * Hardware connection pin on the mootor and the microntroller:
 *   P1.09 -> DM860E PUL+  (STEP signal)
 *   P1.10 -> DM860E DIR+  (Direction)
 *   P1.11 -> DM860E ENA+  (Enable)
 *   GND   -> DM860E PUL-, DIR-, ENA-
 *
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>

/* Main berain of the system  settings - change these to control what the system does */

/* Set to 1 for automatic shuttle testing, 0 for when the dashboard is controlling it */
#define MOTOR_TEST_MODE     1

/* Shuttle test positions in millimeters
 * The motor will move back and forth between these two spots
 *  */
#define SHUTTLE_POS_A       0       /* Starting position (home) */
#define SHUTTLE_POS_B       300     /* How far to travel */

/* How long to pause at each end before turning around (milliseconds) */
#define SHUTTLE_PAUSE_MS    500

/* Motor hardware settings pins connection */
#define GPIO_DEV_NODE           DT_NODELABEL(gpio1)
#define PIN_STEP                9
#define PIN_DIR                 10
#define PIN_ENA                 11

/* Calibration value - how many motor steps equals 1 millimeter of travel
 * Adjusting  this when the motor doesn't move the right distance */
#define STEPS_PER_MM            40

/* Physical limits of the rail (millimeters) since the rail belt is 600mm long this preevnt it to reach end point and strt to vibrate */
#define RAIL_MIN_MM             0
#define RAIL_MAX_MM             590

/* Motor speeds in microseconds per step (smaller = faster)
 * Normal speed for regular moves, slow for careful positioning, homing for finding zero */
#define MOTOR_SPEED_NORMAL_US   500
#define MOTOR_SPEED_SLOW_US     1500
#define MOTOR_SPEED_HOMING_US   2000

/* Acceleration settings - how many steps to ramp up/down speed */
#define MOTOR_ACCEL_STEPS       200

/* Maximum steps to try when homing (safety limit)
 * REDUCED FOR TESTING - homing will finish quickly */
#define HOMING_MAX_STEPS        2000

/* Direction constants */
#define DIR_FORWARD             1
#define DIR_REVERSE             0

static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_DEV_NODE);

/* Solenoid configuration - these settings control the 16 note strikers */
#define TEST_MODE               0
#define SAFE_TEST_MODE          0
#define ENABLE_UART_MODE        0
#define TOTAL_SOLENOIDS         16
#define TEST_SOLENOID_COUNT     16
#define BASE_MIDI_NOTE          60

/* Solenoid timing - kick is the initial strike, then hold keeps it pressed */
#define KICK_DURATION_MS        20
#define MIN_PWM_PERCENT         18
#define TIMER_PERIOD_MS         5

/* PCA9685 PWM controller settings (this chip drives the solenoids) */
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

/*this is  clock system - keeps track of timing for synchronized events */
static uint32_t system_time_offset = 0;

static inline uint32_t get_sync_time(void) {
    return k_uptime_get_32() - system_time_offset;
}

void sync_clock(void) {
    system_time_offset = k_uptime_get_32();
    printk("[SYNC] T=0\n");
}

/* Event system - manages scheduled actions like strikes and moves
 " */

typedef enum {
    EVENT_SYNC,     /* Reset the clock */
    EVENT_STOP,     /* Stop everything */
    EVENT_MOVE,     /* Move the motor */
    EVENT_STRIKE    /* Strike a solenoid */
} event_type_t;

typedef struct {
    event_type_t type;
    uint32_t     target_time_ms;      /* When to execute this event */
    uint8_t      solenoid_number;     /* Which solenoid or position */
    uint8_t      velocity;            /* How hard to strike (0-127) */
    uint16_t     duration_ms;         /* How long to hold */
    uint16_t     sequence_number;     /* For tracking in logs */
    uint8_t      hand;                /* Left or right hand (for motor) */
} kdaa_event_t;

/* Circular buffer to store upcoming events */
#define EVENT_BUFFER_SIZE 256

typedef struct {
    kdaa_event_t      events[EVENT_BUFFER_SIZE];
    volatile uint16_t write_index, read_index, count;
    uint32_t          total_added, total_executed, overflow_count;
} event_buffer_t;

static K_MUTEX_DEFINE(event_mutex);
static event_buffer_t event_buffer;

/* Initialize an empty event buffer */
static void init_event_buffer(event_buffer_t *b) {
    memset(b, 0, sizeof(*b));
}

/* Add an event to the queue (thread-safe) */
static bool add_event(event_buffer_t *b, const kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count >= EVENT_BUFFER_SIZE) {
        b->overflow_count++;
        k_mutex_unlock(&event_mutex);
        return false;
    }
    b->events[b->write_index] = *ev;
    b->write_index = (b->write_index + 1) & (EVENT_BUFFER_SIZE - 1);
    b->count++;
    b->total_added++;
    k_mutex_unlock(&event_mutex);
    return true;
}

/* this rmove and return the next event (thread-safe) */
static bool pop_event(event_buffer_t *b, kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count == 0) {
        k_mutex_unlock(&event_mutex);
        return false;
    }
    *ev = b->events[b->read_index];
    b->read_index = (b->read_index + 1) & (EVENT_BUFFER_SIZE - 1);
    b->count--;
    b->total_executed++;
    k_mutex_unlock(&event_mutex);
    return true;
}

/* this helops me to look at the next event without removing it (thread-safe) */
static bool peek_event(const event_buffer_t *b, kdaa_event_t *ev) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    if (b->count == 0) {
        k_mutex_unlock(&event_mutex);
        return false;
    }
    *ev = b->events[b->read_index];
    k_mutex_unlock(&event_mutex);
    return true;
}

/* this helps me to clear all pending events */
static void clear_events(event_buffer_t *b) {
    k_mutex_lock(&event_mutex, K_FOREVER);
    b->write_index = b->read_index = b->count = 0;
    memset(b->events, 0, sizeof(b->events));
    k_mutex_unlock(&event_mutex);
}

/* Solenoid state tracking - keeps track of each solenoid's current status */
struct Solenoid {
    uint8_t  velocity;            /* Current strike velocity */
    bool     is_active;           /* Is it currently pressed? */
    uint8_t  channel;             /* Which PCA9685 channel */
    bool     in_kick_phase;       /* In the high-power strike phase? */
    uint32_t kick_start_time;     /* When the kick started */
    uint32_t duration_target_ms;  /* How long to hold total */
    uint32_t activation_time;     /* When it was first activated */
};

static struct Solenoid solenoids[TOTAL_SOLENOIDS];

/*this is a  Background threads for sequencer and motor controller */
#define SEQUENCER_STACK_SIZE    4096
#define SEQUENCER_PRIORITY      K_PRIO_COOP(1)
K_THREAD_STACK_DEFINE(sequencer_stack, SEQUENCER_STACK_SIZE);
static struct k_thread sequencer_thread;

/* Motor state tracking the movemnt of the motor fronm one position to another  */
static struct {
    int32_t  position_steps;      /* Current position in motor steps */
    int32_t  position_mm;         /* Current position in millimeters */
    bool     is_homed;            /* Has it found start position? */
    bool     is_moving;           /* Is it moving right now? */
    bool     shuttle_running;     /* Is shuttle mode active? */
} motor_state = { 0, 0, false, false, false };

#define MOTOR_STACK_SIZE    4096
#define MOTOR_PRIORITY      K_PRIO_COOP(2)
K_THREAD_STACK_DEFINE(motor_stack, MOTOR_STACK_SIZE);
static struct k_thread motor_thread;

/* Commands to send to the motor thread */
typedef struct {
    int32_t  target_mm;    /* Where to move to */
    uint32_t speed_us;     /* How fast to go */
} motor_cmd_t;

K_MSGQ_DEFINE(motor_msgq, sizeof(motor_cmd_t), 8, 4);

/* UART communication (for receiving commands from computer) */
#if ENABLE_UART_MODE
#define MSG_SIZE 128
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char rx_buf[MSG_SIZE];
static int  rx_pos = 0;
#endif

/* PCA9685 PWM chip communication functions */

/* Writing  a single register to the PCA9685 */
static int pca9685_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write(i2c_device, buf, 2, PCA9685_I2C_ADDR);
    if (ret) {
        printk("[ERROR] PCA9685 reg=0x%02X val=0x%02X err=%d\n", reg, val, ret);
    }
    return ret;
}

/* Set the PWM value for one channel (controls solenoid power) */
static int pca9685_set_channel(uint8_t ch, uint16_t v) {
    if (ch >= 16) return -EINVAL;
    
    uint8_t reg = PCA9685_LED0_ON_L + (ch * 4);
    uint8_t buf[5];
    buf[0] = reg;
    
    /* Set the on/off times to create the PWM signal */
    if (v == 0) {
        /* Fully off */
        buf[1] = 0x00; buf[2] = 0x00; buf[3] = 0x00; buf[4] = 0x10;
    } else if (v >= PCA9685_MAX_PWM) {
        /* Fully on */
        buf[1] = 0x00; buf[2] = 0x10; buf[3] = 0x00; buf[4] = 0x00;
    } else {
        /* Partial power */
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = (uint8_t)(v & 0xFF);
        buf[4] = (uint8_t)(v >> 8);
    }
    
    int ret = i2c_write(i2c_device, buf, 5, PCA9685_I2C_ADDR);
    if (ret) {
        printk("[ERROR] PCA9685 ch=%d val=%u err=%d\n", ch, v, ret);
    }
    return ret;
}

/* this Turn off all solenoids at once */
static int pca9685_all_off(void) {
    int r = 0;
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_L,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_H,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10);
    return r;
}

/* Convert MIDI velocity (0-127) to PWM value using a quadratic curve
 * This makes the response pressing keys to be more smooth not buzzing nose */
static uint16_t velocity_to_pwm(uint8_t v) {
    if (v == 0) return 0;
    if (v >= 127) return PCA9685_MAX_PWM;
    
    /* Calculate using integer math to avoid floating point */
    uint32_t v2  = (uint32_t)v * v;
    uint32_t pct = MIN_PWM_PERCENT + (v2 * (100U - MIN_PWM_PERCENT)) / (127U * 127U);
    return (uint16_t)((pct * PCA9685_MAX_PWM) / 100U);
}

/* Convert solenoid number to musical note name */
static const char *note_name(uint8_t n) {
    static const char *names[] = {
        "C","C#","D","D#","E","F","F#","G","G#","A","A#","B","C'","C#'","D'","D#'"
    };
    return (n < TOTAL_SOLENOIDS) ? names[n] : "?";
}

/* Initialize solenoid data and print the mapping */
static void init_solenoid_data(void) {
    printk("\n[INIT] Solenoid mapping:\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i] = (struct Solenoid){ .channel = (uint8_t)i };
        printk("  Sol %2d -> CH%2d (MIDI %3d - %s)\n",
               i, i, BASE_MIDI_NOTE + i, note_name(i));
    }
    printk("\n");
}

/* Initialize the PCA9685 PWM chip to connect to the mosfets */
static int init_pca9685(void) {
    printk("[INIT] PCA9685 on i2c21 addr=0x%02X (~203Hz)\n", PCA9685_I2C_ADDR);
    
    if (!device_is_ready(i2c_device)) {
        printk("[ERROR] i2c21 not ready\n");
        return -ENODEV;
    }
    
    /* Put to sleep to set prescaler */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_SLEEP)) return -EIO;
    k_usleep(500);
    
    /* Set PWM frequency */
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VALUE)) return -EIO;
    
    /* Wake up and enable auto-increment */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) return -EIO;
    k_usleep(500);
    
    /* Set output mode */
    if (pca9685_write_reg(PCA9685_MODE2, 0x04)) return -EIO;
    
    /* Making sure everything starts off */
    if (pca9685_all_off()) return -EIO;
    
    printk("[INIT] PCA9685 ready\n");
    return 0;
}

/* Activate a solenoid with specified velocity and duration */
void activate_solenoid(uint8_t sol, uint8_t velocity, uint32_t duration_ms) {
    if (sol >= TOTAL_SOLENOIDS) return;
    
    struct Solenoid *s = &solenoids[sol];
    
    if (velocity > 0) {
        /* Strike the solenoid */
        s->velocity = velocity;
        s->is_active = true;
        s->duration_target_ms = duration_ms;
        s->activation_time = k_uptime_get_32();
        s->kick_start_time = s->activation_time;
        s->in_kick_phase = true;
        
        /* Start with full power for the kick */
        pca9685_set_channel(s->channel, PCA9685_MAX_PWM);
        printk("[KICK] sol=%2d vel=%3d dur=%4dms T=%u\n",
               sol, velocity, duration_ms, s->activation_time);
    } else {
        /* Releases the solenoid */
        s->velocity = 0;
        s->is_active = false;
        s->in_kick_phase = false;
        s->duration_target_ms = 0;
        
        pca9685_set_channel(s->channel, 0);
        printk("[RELEASE] sol=%2d T=%u\n", sol, get_sync_time());
    }
}

/* Turn off all solenoids immediately to prevent burnout  */
void deactivate_all_solenoids(void) {
    printk("[ALL-OFF]\n");
    pca9685_all_off();
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
        solenoids[i].in_kick_phase = false;
        solenoids[i].duration_target_ms = 0;
    }
}

/* Timer callback - runs regularly to manage solenoid transitions
 
static volatile bool pending_actions[TOTAL_SOLENOIDS];

static void control_timer_cb(struct k_timer *timer) {
    uint32_t now = k_uptime_get_32();
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        struct Solenoid *s = &solenoids[i];
        if (!s->is_active) continue;
        
        /* Check if kick phase is over */
        if (s->in_kick_phase && (now - s->kick_start_time) >= KICK_DURATION_MS) {
            s->in_kick_phase = false;
            pending_actions[i] = true;
        }
        
        /* Check if total duration is over */
        if (s->duration_target_ms > 0 && (now - s->activation_time) >= s->duration_target_ms) {
            s->is_active = false;
            pending_actions[i] = true;
        }
    }
}

K_TIMER_DEFINE(control_timer, control_timer_cb, NULL);

/* Process pending solenoid updates (must be called from main thread, not ISR)
 
static void process_solenoid_updates(void) {
    uint32_t now = get_sync_time();
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        if (!pending_actions[i]) continue;
        
        pending_actions[i] = false;
        struct Solenoid *s = &solenoids[i];
        
        if (!s->is_active) {
            /* Time to release this solenoid */
            pca9685_set_channel(s->channel, 0);
            printk("[AUTO-REL] sol=%2d T=%u\n", i, now);
            s->duration_target_ms = 0;
        } else if (!s->in_kick_phase) {
            /* Transition from kick to hold phase */
            uint16_t hold_pwm = velocity_to_pwm(s->velocity);
            pca9685_set_channel(s->channel, hold_pwm);
            printk("[HOLD] sol=%2d pwm=%4u T=%u\n", i, hold_pwm, now);
        }
    }
}


/* Set the motor direction */
static void motor_set_dir(int dir) {
    gpio_pin_set(gpio_dev, PIN_DIR, dir);
    k_usleep(5);  /* Small delay for the driver to register the change */
}

/* Sending a single step pulse singnal to the motor */
static void motor_step_pulse(uint32_t delay_us) {
    gpio_pin_set(gpio_dev, PIN_STEP, 1);
    k_usleep(delay_us);
    gpio_pin_set(gpio_dev, PIN_STEP, 0);
    k_usleep(delay_us);
}

/* Moving the motor a specific number of steps with acceleration/deceleration */
static void motor_run_steps(int dir, uint32_t steps, uint32_t speed_us) {
    if (steps == 0) return;
    
    motor_set_dir(dir);
    motor_state.is_moving = true;
    
    for (uint32_t i = 0; i < steps; i++) {
        uint32_t us = speed_us;
        
        /* Apply acceleration and deceleration for smooth movement */
        if (steps > (uint32_t)(MOTOR_ACCEL_STEPS * 2)) {
            if (i < MOTOR_ACCEL_STEPS) {
                /* Accelerating - start slow and speed up */
                uint32_t f = MOTOR_ACCEL_STEPS - i;
                us = speed_us + (f * speed_us * 2) / MOTOR_ACCEL_STEPS;
            } else if (i > steps - MOTOR_ACCEL_STEPS) {
                /* Decelerating - slow backword */
                uint32_t f = i - (steps - MOTOR_ACCEL_STEPS);
                us = speed_us + (f * speed_us * 2) / MOTOR_ACCEL_STEPS;
            }
        }
        
        motor_step_pulse(us);
        
        /* Update position tracking */
        if (dir == DIR_FORWARD) {
            motor_state.position_steps++;
        } else {
            motor_state.position_steps--;
        }
    }
    
    motor_state.position_mm = motor_state.position_steps / STEPS_PER_MM;
    motor_state.is_moving = false;
}

/* the motor  driving it to the end stop */
static void motor_home(void) {
    printk("[MOTOR] Homing — driving to end stop...\n");
    
    /* Enable the motor driver */
    gpio_pin_set(gpio_dev, PIN_ENA, 1);
    k_msleep(50);
    
    /* Drive slowly backwards until the motor hit the end */
    motor_set_dir(DIR_REVERSE);
    for (uint32_t i = 0; i < HOMING_MAX_STEPS; i++) {
        motor_step_pulse(MOTOR_SPEED_HOMING_US);
    }
    
    k_msleep(200);
    
    /* prevent overshooting in when going forward */
    printk("[MOTOR] Backing off 5mm\n");
    motor_run_steps(DIR_FORWARD, 5 * STEPS_PER_MM, MOTOR_SPEED_HOMING_US);
    
    /* This is now motor zero position */
    motor_state.position_steps = 0;
    motor_state.position_mm = 0;
    motor_state.is_homed = true;
    
    printk("[MOTOR] Homed. Position = 0mm\n");
}

/* Move to an absolute position in millimeters */
static void motor_goto_mm(int32_t target_mm, uint32_t speed_us) {
    /* Keep within rail limits */
    if (target_mm < RAIL_MIN_MM) target_mm = RAIL_MIN_MM;
    if (target_mm > RAIL_MAX_MM) target_mm = RAIL_MAX_MM;
    
    /* Calculate how far to move */
    int32_t  target_steps = (int32_t)(target_mm * STEPS_PER_MM);
    int32_t  delta        = target_steps - motor_state.position_steps;
    
    if (delta == 0) return;  /* Already there */
    
    /* Figure out which direction and how many steps */
    int      dir   = (delta > 0) ? DIR_FORWARD : DIR_REVERSE;
    uint32_t steps = (uint32_t)abs(delta);
    
    printk("[MOTOR] %dmm -> %dmm (%d steps %s)\n",
           motor_state.position_mm, target_mm, steps,
           dir == DIR_FORWARD ? "forward" : "reverse");
    
    motor_run_steps(dir, steps, speed_us);
    
    printk("[MOTOR] Arrived at %dmm\n", motor_state.position_mm);
}

/* 
 
 * In SHUTTLE mode to statrt the movement of the motor (MOTOR_TEST_MODE = 1):
 
 * In KDAA mode (MOTOR_TEST_MODE = 0):
 *   After homing, the motor waits for move commands from the dashboard.
  */
static void motor_thread_fn(void *a1, void *a2, void *a3) {
    printk("[MOTOR] Thread started (priority %d)\n", MOTOR_PRIORITY);
    
    /* Initialize GPIO pins */
    if (!device_is_ready(gpio_dev)) {
        printk("[ERROR] Motor GPIO not ready\n");
        return;
    }
    
    gpio_pin_configure(gpio_dev, PIN_STEP, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_DIR,  GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, PIN_ENA,  GPIO_OUTPUT_INACTIVE);
    
    /* Always starting point  first to find our zero position */
    motor_home();
    
#if MOTOR_TEST_MODE
    /* Shuttle test mode - automatic back and forth movement */
    motor_state.shuttle_running = true;
    bool at_a = true;
    uint32_t pass = 0;
    
    printk("[MOTOR] Shuttle mode started\n");
    printk("[MOTOR] Range: %dmm <-> %dmm\n", SHUTTLE_POS_A, SHUTTLE_POS_B);
    printk("[MOTOR] Pause at each end: %dms\n", SHUTTLE_PAUSE_MS);
    printk("[MOTOR] Running forever — power off to stop\n\n");
    
    while (1) {
        if (at_a) {
            /* Go to position B */
            printk("[MOTOR] Pass %u — going to %dmm\n", pass, SHUTTLE_POS_B);
            motor_goto_mm(SHUTTLE_POS_B, MOTOR_SPEED_NORMAL_US);
            at_a = false;
        } else {
            /* Return to position A */
            printk("[MOTOR] Pass %u — returning to %dmm\n", pass, SHUTTLE_POS_A);
            motor_goto_mm(SHUTTLE_POS_A, MOTOR_SPEED_NORMAL_US);
            at_a = true;
            pass++;
        }
        
        /* Pause before reversing */
        printk("[MOTOR] Pausing %dms at %dmm\n", SHUTTLE_PAUSE_MS, motor_state.position_mm);
        k_msleep(SHUTTLE_PAUSE_MS);
    }
    
#else
    /* KDAA mode - wait for commands from the dashboard */
    motor_cmd_t cmd;
    while (1) {
        if (k_msgq_get(&motor_msgq, &cmd, K_FOREVER) == 0) {
            printk("[MOTOR] CMD -> %dmm\n", cmd.target_mm);
            motor_goto_mm(cmd.target_mm, cmd.speed_us);
        }
    }
#endif
}

/* Queue a motor move command (used by the event system) */
static void motor_queue_move(int32_t target_mm, uint32_t speed_us) {
    motor_cmd_t cmd = { .target_mm = target_mm, .speed_us = speed_us };
    if (k_msgq_put(&motor_msgq, &cmd, K_NO_WAIT) != 0) {
        printk("[WARN] Motor queue full\n");
    }
}

/* Sequencer thread - processes scheduled events at the right time */
static void sequencer_loop(void *a1, void *a2, void *a3) {
    kdaa_event_t ev;
    printk("[SEQ] Started prio=%d poll=0.5ms\n", SEQUENCER_PRIORITY);
    
    while (1) {
        uint32_t now = get_sync_time();
        
        /* Check if any events are ready to execute */
        while (peek_event(&event_buffer, &ev)) {
            if (now < ev.target_time_ms) break;  /* Not time yet */
            
            pop_event(&event_buffer, &ev);
            
            /* Execute the event based on its type */
            switch (ev.type) {
            case EVENT_STRIKE:
                if (ev.solenoid_number < TOTAL_SOLENOIDS) {
                    activate_solenoid(ev.solenoid_number, ev.velocity, ev.duration_ms);
                    printk("[EXEC] #%u STRIKE sol=%2d vel=%3d dur=%4dms delta=%d\n",
                           ev.sequence_number, ev.solenoid_number, ev.velocity,
                           ev.duration_ms, (int)(now - ev.target_time_ms));
                }
                break;
                
            case EVENT_STOP:
                printk("[EXEC] #%u STOP\n", ev.sequence_number);
                clear_events(&event_buffer);
                deactivate_all_solenoids();
                break;
                
            case EVENT_SYNC:
                sync_clock();
                break;
                
            case EVENT_MOVE:
                printk("[EXEC] #%u MOVE hand=%c pos=%dmm\n",
                       ev.sequence_number, (char)ev.hand, ev.solenoid_number);
                motor_queue_move((int32_t)ev.solenoid_number, MOTOR_SPEED_NORMAL_US);
                break;
                
            default:
                break;
            }
        }
        
        /* Small delay before checking again */
        k_usleep(500);
    }
}

/* UART handling - receives commands from the dashboard */
#if ENABLE_UART_MODE

/* UART interrupt callback - receives characters */
static void uart_cb(const struct device *dev, void *user_data) {
    uint8_t c;
    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev)) return;
    
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (c == '\n' || c == '\r') {
            /* End of command - process it */
            if (rx_pos > 0) {
                rx_buf[rx_pos] = '\0';
                k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
                rx_pos = 0;
            }
        } else if (rx_pos < (MSG_SIZE - 1)) {
            /* Add character to buffer */
            rx_buf[rx_pos++] = c;
        } else {
            /* Buffer overflow - reset */
            rx_pos = 0;
        }
    }
}

/* Parse and execute KDAA protocol commands */
static void parse_kdaa(char *msg) {
    int note, vel, t_ms, dur;
    char hand;
    
    /* SYNC command - reset the clock */
    if (strncmp(msg, "SYNC", 4) == 0) {
        sync_clock();
        return;
    }
    
    /* STOP command - emergency stop */
    if (strcmp(msg, "STOP") == 0) {
        deactivate_all_solenoids();
        init_event_buffer(&event_buffer);
        return;
    }
    
    /* M: command - move motor (format: M:H:position:time) */
    if (msg[0] == 'M' && msg[1] == ':') {
        int p;
        uint32_t t;
        if (sscanf(msg, "M:%c:%d:%u", &hand, &p, &t) == 3) {
            kdaa_event_t ev = {
                .type = EVENT_MOVE,
                .target_time_ms = t,
                .solenoid_number = (uint8_t)p,
                .hand = (uint8_t)hand
            };
            add_event(&event_buffer, &ev);
        }
        return;
    }
    
    /* S: command - strike solenoid (format: S:H:note:velocity:time:duration) */
    if (msg[0] == 'S' && msg[1] == ':') {
        if (sscanf(msg, "S:%c:%d:%d:%d:%d", &hand, &note, &vel, &t_ms, &dur) == 5) {
            if (note >= 0 && note <= 127 && vel >= 0 && vel <= 127) {
                kdaa_event_t ev = {
                    .type = EVENT_STRIKE,
                    .target_time_ms = (uint32_t)t_ms,
                    .solenoid_number = (uint8_t)(note - BASE_MIDI_NOTE),
                    .velocity = (uint8_t)vel,
                    .duration_ms = (uint16_t)dur
                };
                add_event(&event_buffer, &ev);
            }
        }
        return;
    }
    
    printk("[ERR] Use S:H:N:V:T:D or M:H:P:T or SYNC or STOP\n");
}

#endif

/* Boot test sequence - fires all solenoids on startup to verify if they work */

/* Helping  function to add a strike event to the queue */
static void queue_strike(uint16_t seq, uint32_t t, uint8_t sol, uint8_t vel, uint16_t dur) {
    kdaa_event_t ev = {
        .type = EVENT_STRIKE,
        .target_time_ms = t,
        .solenoid_number = sol,
        .velocity = vel,
        .duration_ms = dur,
        .sequence_number = seq
    };
    if (!add_event(&event_buffer, &ev)) {
        printk("[ERR] Buffer full\n");
    }
}

/* Run the boot test to verify two passes through all solenoids */
static void run_solenoid_test(void) {
    printk("\n[TEST] Solenoid boot sequence (%d solenoids)\n", TEST_SOLENOID_COUNT);
    
    uint16_t seq = 1;
    
    /* First pass - all solenoids at full power, 500ms apart */
    for (int i = 0; i < TEST_SOLENOID_COUNT; i++) {
        queue_strike(seq++, (uint32_t)(500 + i * 500), (uint8_t)i, 127, 120);
    }
    
    /* Second pass - same thing to verify consistency */
    for (int i = 0; i < TEST_SOLENOID_COUNT; i++) {
        queue_strike(seq++, (uint32_t)(8500 + i * 500), (uint8_t)i, 127, 100);
    }
    
    /* Auto-stop after the test */
    kdaa_event_t stop = {
        .type = EVENT_STOP,
        .target_time_ms = 16500,
        .sequence_number = seq++
    };
    add_event(&event_buffer, &stop);
    
    printk("[TEST] %u events queued\n\n", seq - 1);
}

/*  system initialization and main loop */
int main(void) {
    printk("\n=== KLAVIATOR V4.0 ===\n");
    
#if MOTOR_TEST_MODE
    printk("Mode: MOTOR SHUTTLE TEST (%dmm <-> %dmm)\n\n", SHUTTLE_POS_A, SHUTTLE_POS_B);
#else
    printk("Mode: Solenoid + Motor KDAA\n\n");
#endif
    
    k_msleep(100);
    
    /* Initialize the solenoid system */
    if (init_pca9685() < 0) {
        printk("[WARN] PCA9685 failed\n");
    }
    init_solenoid_data();
    sync_clock();
    
    /* the solenoid control timer */
    k_timer_start(&control_timer, K_MSEC(TIMER_PERIOD_MS), K_MSEC(TIMER_PERIOD_MS));
    printk("[INIT] Control timer started\n");
    
    init_event_buffer(&event_buffer);
    
    /* the sequencer thread that  executes scheduled events */
    k_thread_create(&sequencer_thread, sequencer_stack,
                    K_THREAD_STACK_SIZEOF(sequencer_stack),
                    sequencer_loop, NULL, NULL, NULL,
                    SEQUENCER_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&sequencer_thread, "sequencer");
    printk("[INIT] Sequencer started\n");
    
    /* the thread to start motor control */
    k_thread_create(&motor_thread, motor_stack,
                    K_THREAD_STACK_SIZEOF(motor_stack),
                    motor_thread_fn, NULL, NULL, NULL,
                    MOTOR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&motor_thread, "motor");
    printk("[INIT] Motor thread started\n");
    
    printk("[READY] System ready\n\n");
    
    /* Run the boot test sequence */
    // run_solenoid_test();  /* this Disables only the motor is being test independentkly  */
    
#if ENABLE_UART_MODE
    /* Set up UART communication */
    if (device_is_ready(uart_dev)) {
        uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
        uart_irq_rx_enable(uart_dev);
    }
    
    char msg[MSG_SIZE];
    uint32_t hb = 0;
    
    /*  loop that  process commands and update solenoids */
    while (1) {
        process_solenoid_updates();
        
        if (k_msgq_get(&uart_msgq, msg, K_MSEC(1)) == 0) {
            parse_kdaa(msg);
        } else if ((hb % 1000) == 0) {
            printk("[HB] alive %u  motor=%dmm\n", hb, motor_state.position_mm);
        }
        
        hb++;
        k_msleep(1);
    }
#else
    /* Main loop without UART connection */
    uint32_t n = 0;
    
    while (1) {
        process_solenoid_updates();
        
        /* Print heartbeat occasionally so I know the system is running and functioning as its suppose to */
        if ((n % 1000) == 0) {
            printk("[HB] alive %u  motor=%dmm\n", n, motor_state.position_mm);
        }
        
        n++;
        k_msleep(1);
    }
#endif
    
    return 0;
}
