/*
 * ===============================================================================
 * KLAVIATOR - Direct Drive 16 Solenoid Controller V4.0
 * ===============================================================================
 *
 * Features:
 *   - 1:1 Direct control (P1.0-P1.15)
 *   - Kick-to-Hold transition (100% kick → velocity hold)
 *   - Duration-based auto-release
 *   - Clock synchronization for sequencer timing
 *   - KDAA V4.0 protocol: S:H:N:V:T:D
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

/* ===============================================================================
 * CONFIGURATION SETTINGS
 * =============================================================================== */

#define ENABLE_UART_MODE        1

#define TOTAL_SOLENOIDS         16
#define BASE_MIDI_NOTE          60

#define PWM_FREQUENCY_HZ        1000
#define PWM_PERIOD_USEC         (1000000UL / PWM_FREQUENCY_HZ)

#define KICK_DURATION_MS        20      /* Duration of 100% kick phase */
#define MIN_PWM_PERCENT         18

#define TIMER_PERIOD_MS         5       /* Run control timer every 5ms */

/* ===============================================================================
 * HARDWARE DEFINITIONS
 * =============================================================================== */

#define PWM_DEV_NODE DT_NODELABEL(pwm20)
static const struct device *pwm_device = DEVICE_DT_GET(PWM_DEV_NODE);

#define GPIO_DEV_NODE DT_NODELABEL(gpio1)
static const struct device *gpio_device = DEVICE_DT_GET(GPIO_DEV_NODE);

#define NUM_PWM_CHANNELS    4   /* nRF54L15 PWM20 has only 4 channels */

/* ===============================================================================
 * CLOCK SYNCHRONIZATION
 * =============================================================================== */

static uint32_t system_time_offset = 0;

/* Get current synchronized time in milliseconds */
static inline uint32_t get_sync_time(void)
{
    return k_uptime_get_32() - system_time_offset;
}

/* Reset clock synchronization (called from Python Dashboard) */
void sync_clock(void)
{
    system_time_offset = k_uptime_get_32();
    printk("[SYNC] Clock synchronized. T=0\n");
}

/* ===============================================================================
 * DATA STRUCTURES
 * =============================================================================== */

struct Solenoid {
    uint8_t velocity;           /* Target hold velocity (0-127) */
    bool is_active;             /* Currently energized */
    uint32_t pin;               /* Direct PWM pin (0-15 for P1.0-P1.15) */
    
    /* Kick-to-hold transition */
    bool in_kick_phase;         /* Currently in 100% kick */
    uint32_t kick_start_time;   /* When kick started (sync time) */
    
    /* Duration control */
    uint32_t duration_target_ms; /* How long to stay on (0 = manual control) */
    uint32_t activation_time;    /* When activated (sync time) */
};

static struct Solenoid solenoids[TOTAL_SOLENOIDS];

/* ===============================================================================
 * UART COMMUNICATION
 * =============================================================================== */

#if ENABLE_UART_MODE
#define MSG_SIZE 128
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;
#endif

/* ===============================================================================
 * HELPER FUNCTIONS
 * =============================================================================== */

static uint32_t velocity_to_pwm_pulse(uint8_t velocity)
{
    if (velocity == 0) return 0;
    if (velocity >= 127) return PWM_NSEC(PWM_PERIOD_USEC * 1000);

    float norm = (float)velocity / 127.0f;
    float curved = norm * norm;
    float pwm_percent = MIN_PWM_PERCENT + (curved * (100.0f - MIN_PWM_PERCENT));

    uint32_t pulse_ns = (uint32_t)((pwm_percent * PWM_PERIOD_USEC * 1000) / 100.0f);
    return PWM_NSEC(pulse_ns);
}

static const char* get_note_name(uint8_t solenoid_number)
{
    static const char *note_names[] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G",
        "G#", "A", "A#", "B", "C'", "C#'", "D'", "D#'"
    };
    return (solenoid_number < TOTAL_SOLENOIDS) ? note_names[solenoid_number] : "??";
}

/* ===============================================================================
 * INITIALIZATION
 * =============================================================================== */

static void initialize_solenoid_data(void)
{
    printk("\n[INIT] Initializing hybrid drive solenoid mappings...\n");
    printk("[INIT] Solenoids 0-3: PWM (P1.0-P1.3) - Variable power\n");
    printk("[INIT] Solenoids 4-15: GPIO (P1.4-P1.12, P1.15-P1.17) - On/Off only\n\n");

    /* Map GPIO pins for solenoids 4-15 (avoiding P1.13, P1.14 used by UART) */
    uint8_t gpio_pins[] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 16, 17};
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
        
        if (i < NUM_PWM_CHANNELS) {
            solenoids[i].pin = i;  /* PWM channels 0-3 → P1.0-P1.3 */
        } else {
            solenoids[i].pin = gpio_pins[i - NUM_PWM_CHANNELS];  /* GPIO pins */
        }
        
        solenoids[i].in_kick_phase = false;
        solenoids[i].kick_start_time = 0;
        solenoids[i].duration_target_ms = 0;
        solenoids[i].activation_time = 0;

        const char *type = (i < NUM_PWM_CHANNELS) ? "PWM" : "GPIO";
        printk("  Solenoid %2d: %s P1.%02d, Note %3d (%s)\n",
               i, type, solenoids[i].pin, BASE_MIDI_NOTE + i, get_note_name(i));
    }
    printk("[INIT] Solenoid mapping completed\n");
}

static int initialize_pwm(void)
{
    printk("\n[INIT] Initializing PWM controller (%d Hz)...\n", PWM_FREQUENCY_HZ);

    if (!device_is_ready(pwm_device)) {
        printk("[ERROR] PWM device not ready\n");
        return -1;
    }

    /* Initialize 4 PWM channels to OFF */
    for (int ch = 0; ch < NUM_PWM_CHANNELS; ch++) {
        pwm_set(pwm_device, ch, PWM_NSEC(PWM_PERIOD_USEC * 1000), 0, 0);
    }

    printk("[INIT] PWM initialized at %d Hz on P1.0-P1.3\n", PWM_FREQUENCY_HZ);
    return 0;
}

static int initialize_gpio(void)
{
    printk("\n[INIT] Initializing GPIO pins for solenoids 4-15...\n");

    if (!device_is_ready(gpio_device)) {
        printk("[ERROR] GPIO device (gpio1) not ready\n");
        return -1;
    }

    /* Initialize GPIO pins for solenoids 4-15 */
    for (int i = NUM_PWM_CHANNELS; i < TOTAL_SOLENOIDS; i++) {
        gpio_pin_configure(gpio_device, solenoids[i].pin, GPIO_OUTPUT_INACTIVE);
    }

    printk("[INIT] GPIO pins configured on P1.4-P1.12, P1.15-P1.17\n");
    return 0;
}

/* ===============================================================================
 * CONTROL FUNCTIONS
 * =============================================================================== */

void activate_solenoid(uint8_t solenoid_number, uint8_t velocity, uint32_t duration_ms)
{
    if (solenoid_number >= TOTAL_SOLENOIDS) return;

    struct Solenoid *sol = &solenoids[solenoid_number];
    
    if (velocity > 0) {
        /* ACTIVATE */
        sol->velocity = velocity;
        sol->is_active = true;
        sol->duration_target_ms = duration_ms;
        sol->activation_time = get_sync_time();
        
        if (solenoid_number < NUM_PWM_CHANNELS) {
            /* PWM control (solenoids 0-3) with KICK phase */
            sol->in_kick_phase = true;
            sol->kick_start_time = get_sync_time();
            
            /* Set PWM to 100% for KICK */
            uint32_t period = PWM_NSEC(PWM_PERIOD_USEC * 1000);
            uint32_t pulse = period;  /* 100% duty cycle */
            pwm_set(pwm_device, sol->pin, period, pulse, 0);
            
            printk("[KICK-PWM] Sol:%2d | Pin:P1.%02d | Vel:%3d | Dur:%4dms | T:%6u\n",
                   solenoid_number, sol->pin, velocity, duration_ms, sol->activation_time);
        } else {
            /* GPIO control (solenoids 4-15) - Simple ON */
            sol->in_kick_phase = false;  /* No kick phase for GPIO */
            gpio_pin_set(gpio_device, sol->pin, 1);
            
            printk("[ON-GPIO] Sol:%2d | Pin:P1.%02d | Dur:%4dms | T:%6u\n",
                   solenoid_number, sol->pin, duration_ms, sol->activation_time);
        }
    } else {
        /* RELEASE */
        sol->velocity = 0;
        sol->is_active = false;
        sol->in_kick_phase = false;
        sol->duration_target_ms = 0;
        
        if (solenoid_number < NUM_PWM_CHANNELS) {
            /* PWM OFF */
            uint32_t period = PWM_NSEC(PWM_PERIOD_USEC * 1000);
            pwm_set(pwm_device, sol->pin, period, 0, 0);
        } else {
            /* GPIO OFF */
            gpio_pin_set(gpio_device, sol->pin, 0);
        }
        
        const char *type = (solenoid_number < NUM_PWM_CHANNELS) ? "PWM" : "GPIO";
        printk("[RELEASE-%s] Sol:%2d | Pin:P1.%02d | T:%6u\n",
               type, solenoid_number, sol->pin, get_sync_time());
    }
}

void deactivate_solenoid(uint8_t solenoid_number)
{
    activate_solenoid(solenoid_number, 0, 0);
}

void deactivate_all_solenoids(void)
{
    printk("[ALL OFF] Turning off ALL solenoids\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        deactivate_solenoid(i);
    }
}

/* ===============================================================================
 * CONTROL TIMER - Handles kick-to-hold and duration-based release
 * =============================================================================== */

void control_timer_handler(struct k_timer *timer)
{
    uint32_t now = get_sync_time();
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        struct Solenoid *sol = &solenoids[i];
        
        if (!sol->is_active) continue;
        
        /* Check if kick phase is complete → transition to HOLD (PWM only) */
        if (sol->in_kick_phase && i < NUM_PWM_CHANNELS) {
            uint32_t kick_elapsed = now - sol->kick_start_time;
            if (kick_elapsed >= KICK_DURATION_MS) {
                /* Transition to HOLD phase */
                sol->in_kick_phase = false;
                
                uint32_t pulse = velocity_to_pwm_pulse(sol->velocity);
                uint32_t period = PWM_NSEC(PWM_PERIOD_USEC * 1000);
                pwm_set(pwm_device, sol->pin, period, pulse, 0);
                
                int duty_percent = (sol->velocity * 100) / 127;
                printk("[HOLD-PWM] Sol:%2d | Pin:P1.%02d | Duty:%3d%% | T:%6u\n",
                       i, sol->pin, duty_percent, now);
            }
        }
        
        /* Check if duration has elapsed → auto-release (all solenoids) */
        if (sol->duration_target_ms > 0) {
            uint32_t active_time = now - sol->activation_time;
            if (active_time >= sol->duration_target_ms) {
                const char *type = (i < NUM_PWM_CHANNELS) ? "PWM" : "GPIO";
                printk("[AUTO-REL-%s] Sol:%2d | Dur:%4dms | T:%6u\n",
                       type, i, sol->duration_target_ms, now);
                deactivate_solenoid(i);
            }
        }
    }
}

K_TIMER_DEFINE(control_timer, control_timer_handler, NULL);

/* ===============================================================================
 * UART COMMUNICATION - KDAA V4.0 Protocol
 * =============================================================================== */

#if ENABLE_UART_MODE
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    if (!uart_irq_update(uart_dev)) return;
    if (!uart_irq_rx_ready(uart_dev)) return;
    
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            rx_buf[rx_buf_pos] = '\0';
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
            rx_buf_pos = 0;
        } else if (c != '\n' && c != '\r') {
            rx_buf[rx_buf_pos++] = c;
            if (rx_buf_pos >= (MSG_SIZE - 1)) {
                rx_buf_pos = 0;
            }
        }
    }
}

void parse_kdaa_cmd(char *msg)
{
    /* V4.0 Protocol: S:H:N:V:T:D
     * S = Sequence number (ignored for now)
     * H = Hand/group (ignored for now)
     * N = Note (MIDI note)
     * V = Velocity (0-127)
     * T = Target time (ms) - for future sequencer sync
     * D = Duration (ms)
     */
    
    int seq, hand, note, velocity, target_time, duration;
    
    /* Handle sync command */
    if (strcmp(msg, "sync") == 0 || strcmp(msg, "SYNC") == 0) {
        sync_clock();
        return;
    }
    
    /* Handle all off command */
    if (strcmp(msg, "all:0") == 0 || strcmp(msg, "ALL:0") == 0) {
        deactivate_all_solenoids();
        return;
    }
    
    /* Parse V4.0 command: S:H:N:V:T:D */
    if (sscanf(msg, "%d:%d:%d:%d:%d:%d", &seq, &hand, &note, &velocity, &target_time, &duration) == 6) {
        int solenoid = note - BASE_MIDI_NOTE;
        
        if (solenoid < 0 || solenoid >= TOTAL_SOLENOIDS) {
            printk("[ERROR] Note %d out of range (valid: %d-%d)\n",
                   note, BASE_MIDI_NOTE, BASE_MIDI_NOTE + TOTAL_SOLENOIDS - 1);
            return;
        }
        
        if (velocity < 0 || velocity > 127) {
            printk("[ERROR] Velocity must be 0-127\n");
            return;
        }
        
        if (duration < 0) {
            printk("[ERROR] Duration must be >= 0\n");
            return;
        }
        
        activate_solenoid(solenoid, velocity, duration);
        return;
    }
    
    /* Parse simple command: NOTE:VELOCITY (backward compatibility) */
    if (sscanf(msg, "%d:%d", &note, &velocity) == 2) {
        int solenoid = note - BASE_MIDI_NOTE;
        
        if (solenoid >= 0 && solenoid < TOTAL_SOLENOIDS && velocity >= 0 && velocity <= 127) {
            activate_solenoid(solenoid, velocity, 0);  /* No duration = manual control */
        } else {
            printk("[ERROR] Invalid note or velocity\n");
        }
        return;
    }
    
    printk("[ERROR] Invalid command format\n");
    printk("  V4.0: S:H:N:V:T:D (e.g., 1:0:60:100:0:200)\n");
    printk("  Simple: NOTE:VELOCITY (e.g., 60:100)\n");
}
#endif

/* ===============================================================================
 * MAIN
 * =============================================================================== */

int main(void)
{
    printk("\n");
    printk("╔═══════════════════════════════════════════════════════╗\n");
    printk("║  KLAVIATOR V4.0 - Hybrid Drive Controller            ║\n");
    printk("║  PWM (0-3) + GPIO (4-15) | Kick-Hold | Duration      ║\n");
    printk("╚═══════════════════════════════════════════════════════╝\n");
    printk("\n");

    if (initialize_pwm() < 0 || initialize_gpio() < 0) {
        printk("[ERROR] Hardware initialization failed!\n");
        return -1;
    }

    initialize_solenoid_data();
    
    /* Sync clock to T=0 */
    sync_clock();

    /* Start control timer (runs every 5ms) */
    printk("[INIT] Starting control timer (%dms period)...\n", TIMER_PERIOD_MS);
    printk("[INIT] Kick duration: %dms\n", KICK_DURATION_MS);
    k_timer_start(&control_timer, K_MSEC(TIMER_PERIOD_MS), K_MSEC(TIMER_PERIOD_MS));

    printk("[READY] System ready\n\n");

    #if ENABLE_UART_MODE
    printk("╔═══════════════════════════════════════════════════════╗\n");
    printk("║  UART Control Mode - KDAA V4.0 Protocol              ║\n");
    printk("╚═══════════════════════════════════════════════════════╝\n\n");
    
    printk("Command formats:\n");
    printk("  V4.0 Full:  S:H:N:V:T:D\n");
    printk("    Example: 1:0:60:100:0:200\n");
    printk("    → Seq 1, Hand 0, Note 60, Vel 100, Time 0ms, Duration 200ms\n\n");
    
    printk("  Simple:     NOTE:VELOCITY\n");
    printk("    Example: 60:100  (manual control, no auto-release)\n\n");
    
    printk("  Special:    sync     → Reset clock to T=0\n");
    printk("              all:0    → Turn off all solenoids\n\n");
    
    printk("Valid notes: %d-%d (solenoids 0-15)\n", 
           BASE_MIDI_NOTE, BASE_MIDI_NOTE + TOTAL_SOLENOIDS - 1);
    printk("Valid velocity: 0-127\n\n");
    
    if (!device_is_ready(uart_dev)) {
        printk("[ERROR] UART not ready\n");
        return -1;
    }
    
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    
    char msg_buf[MSG_SIZE];
    while (1) {
        if (k_msgq_get(&uart_msgq, &msg_buf, K_FOREVER) == 0) {
            parse_kdaa_cmd(msg_buf);
        }
    }
    
    #else
    printk("UART mode disabled. System idle.\n");
    while (1) {
        k_msleep(1000);
    }
    #endif

    return 0;
}
