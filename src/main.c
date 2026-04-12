/*
 * ═══════════════════════════════════════════════════════════════════
 * KLAVIATOR - 16 Solenoid Controller with Time Multiplexing
 * ═══════════════════════════════════════════════════════════════════
 * 
 * Uses 4 PWM channels to control 16 solenoids via time multiplexing
 * 
 * Architecture:
 *   - 4 PWM pins (P0.1-P0.4): Each controls group of 4 solenoids
 *   - 2 Enable pins (P0.5-P0.6): Select which of 4 solenoids in each group
 *   - Timer: Cycles through the 4 groups rapidly
 * 
 * Example: To control Solenoid 5:
 *   - It's in Group 1 (PWM channel 1)
 *   - It's the 2nd solenoid in that group
 *   - Set enable pins to select position 1 (01 in binary)
 *   - Set PWM channel 1 to desired power level
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

/* ═══════════════════════════════════════════════════════════════════
 * CONFIGURATION SETTINGS
 * ═══════════════════════════════════════════════════════════════════ */

// Set to 1 for UART control, 0 for automatic hardware test
#define ENABLE_UART_MODE 0

// Total number of solenoids we can control
#define TOTAL_SOLENOIDS 16

// How many solenoids each PWM channel controls
#define SOLENOIDS_PER_PWM_CHANNEL 4

// Number of PWM channels available
#define NUM_PWM_CHANNELS 4

// Base MIDI note (Middle C)
#define BASE_MIDI_NOTE 60

// PWM frequency (25 kHz - above audible range for silent operation)
#define PWM_FREQUENCY_HZ 25000
#define PWM_PERIOD_USEC (1000000 / PWM_FREQUENCY_HZ)  // 40µs

// Time multiplexing speed (how fast we switch between groups)
#define MULTIPLEX_FREQUENCY_HZ 200  // Switch every 5ms
#define MULTIPLEX_PERIOD_MS (1000 / MULTIPLEX_FREQUENCY_HZ)

// Kick + Hold timing (for efficient solenoid strikes)
// Count in multiplex CYCLES, not milliseconds (more reliable than time-based)
#define KICK_CYCLES 1           // Number of multiplex cycles at full power (1-2 recommended)
#define HOLD_POWER_PERCENT 60   // Holding power as % of requested velocity

// Auto-release safety timer
#define MAX_SOLENOID_ON_TIME_MS 2000  // Maximum time a solenoid can stay on (2 seconds)
#define ENABLE_AUTO_RELEASE 1          // Set to 1 to enable auto-release safety feature

/* ═══════════════════════════════════════════════════════════════════
 * HARDWARE DEFINITIONS
 * ═══════════════════════════════════════════════════════════════════ */

// PWM device
#define PWM_DEV_NODE DT_NODELABEL(pwm20)
static const struct device *pwm_device = DEVICE_DT_GET(PWM_DEV_NODE);

// GPIO device for enable pins
#define GPIO_DEV_NODE DT_NODELABEL(gpio0)
static const struct device *gpio_device = DEVICE_DT_GET(GPIO_DEV_NODE);

// Enable/Select pins (choose which of 4 solenoids in each group)
#define ENABLE_PIN_0  5  // P0.5 - Selects between solenoid 0/2 or 1/3 in each group
#define ENABLE_PIN_1  6  // P0.6 - Selects between solenoid 0/1 or 2/3 in each group

/* ═══════════════════════════════════════════════════════════════════
 * DATA STRUCTURES
 * ═══════════════════════════════════════════════════════════════════ */

// Information about each solenoid
struct Solenoid {
    uint8_t velocity;          // Power level (0-127, MIDI standard)
    bool is_active;            // Should this solenoid be energized?
    uint8_t pwm_channel;       // Which PWM channel controls this solenoid (0-3)
    uint8_t position_in_group; // Which position within the group (0-3)
    uint16_t cycles_active;    // How many multiplex cycles this solenoid has been active
    bool in_kick_phase;        // True during initial kick, false during hold
};

// Array holding state of all 16 solenoids
static struct Solenoid solenoids[TOTAL_SOLENOIDS];

// Current multiplexing state (which of the 4 groups is active: 0, 1, 2, or 3)
static uint8_t current_multiplex_group = 0;

/* ═══════════════════════════════════════════════════════════════════
 * UART COMMUNICATION (Optional, for remote control)
 * ═══════════════════════════════════════════════════════════════════ */

#if ENABLE_UART_MODE
#define MSG_SIZE 64
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;
#endif

/* ═══════════════════════════════════════════════════════════════════
 * HELPER FUNCTIONS - Easy to understand, descriptive names
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Convert MIDI velocity (0-127) to PWM pulse width
 *
 * Uses a non-linear curve for piano-like dynamics:
 * - Minimum threshold ensures weak notes still activate
 * - Quadratic curve provides better expression and dynamic range
 * - More noticeable difference between soft, medium, and loud
 *
 * @param velocity MIDI velocity value (0 = off, 127 = maximum power)
 * @return PWM pulse width in microseconds
 */
static uint32_t velocity_to_pwm_pulse(uint8_t velocity)
{
    if (velocity == 0) {
        return 0;  // Completely off
    }
    if (velocity >= 127) {
        return PWM_USEC(PWM_PERIOD_USEC);  // Full power
    }
    
    // Piano-like velocity curve with better dynamics
    // Min PWM: 25% (ensures soft notes work)
    // Max PWM: 100% (full power)
    // Curve: Quadratic (velocity²) for expressive dynamics
    
    #define MIN_PWM_PERCENT 25  // Minimum PWM for weakest note
    #define MAX_PWM_PERCENT 100 // Maximum PWM for strongest note
    
    // Normalize velocity to 0.0 - 1.0 range
    float vel_normalized = (float)velocity / 127.0f;
    
    // Apply quadratic curve for better feel
    // This makes soft notes softer and creates more dynamic range
    float vel_curved = vel_normalized * vel_normalized;
    
    // Map to PWM range (25% - 100%)
    float pwm_percent = MIN_PWM_PERCENT + (vel_curved * (MAX_PWM_PERCENT - MIN_PWM_PERCENT));
    
    // Convert percentage to pulse width
    uint32_t pulse_usec = (uint32_t)((pwm_percent * PWM_PERIOD_USEC) / 100.0f);
    
    return PWM_USEC(pulse_usec);
}

/**
 * Get human-readable name for a solenoid based on MIDI note
 * 
 * @param solenoid_number Which solenoid (0-15)
 * @return String like "C", "C#", "D", etc.
 */
static const char* get_note_name(uint8_t solenoid_number)
{
    const char *note_names[] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", 
        "G#", "A", "A#", "B", "C'", "C#'", "D'", "D#'"
    };
    
    if (solenoid_number < TOTAL_SOLENOIDS) {
        return note_names[solenoid_number];
    }
    return "??";
}

/**
 * Calculate which PWM channel controls a given solenoid
 * 
 * Example: Solenoid 5 is controlled by PWM channel 1 (because 5/4 = 1)
 * 
 * @param solenoid_number Which solenoid (0-15)
 * @return PWM channel number (0-3)
 */
static uint8_t get_pwm_channel_for_solenoid(uint8_t solenoid_number)
{
    return solenoid_number / SOLENOIDS_PER_PWM_CHANNEL;
}

/**
 * Calculate position of solenoid within its PWM group
 * 
 * Example: Solenoid 5 is at position 1 in its group (because 5%4 = 1)
 * 
 * @param solenoid_number Which solenoid (0-15)
 * @return Position within group (0-3)
 */
static uint8_t get_position_in_group(uint8_t solenoid_number)
{
    return solenoid_number % SOLENOIDS_PER_PWM_CHANNEL;
}

/* ═══════════════════════════════════════════════════════════════════
 * INITIALIZATION FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Initialize all solenoid data structures
 * Sets up the mapping between solenoids, PWM channels, and positions
 */
static void initialize_solenoid_data(void)
{
    printk("\n📋 Initializing solenoid mappings...\n");
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
        solenoids[i].pwm_channel = get_pwm_channel_for_solenoid(i);
        solenoids[i].position_in_group = get_position_in_group(i);
        solenoids[i].cycles_active = 0;
        solenoids[i].in_kick_phase = false;
        
        printk("  Solenoid %2d: PWM Ch %d, Position %d, Note %3d (%s)\n",
               i,
               solenoids[i].pwm_channel,
               solenoids[i].position_in_group,
               BASE_MIDI_NOTE + i,
               get_note_name(i));
    }
    
    printk("✓ All solenoids initialized\n");
}

/**
 * Initialize GPIO pins used for enable/select signals
 * These pins choose which of the 4 solenoids in each group is active
 */
static int initialize_enable_pins(void)
{
    printk("\n🔌 Setting up enable pins...\n");
    
    if (!device_is_ready(gpio_device)) {
        printk("❌ ERROR: GPIO device not ready\n");
        return -1;
    }
    
    // Configure enable pin 0 (P0.5)
    int ret = gpio_pin_configure(gpio_device, ENABLE_PIN_0, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("❌ ERROR: Failed to configure enable pin 0\n");
        return ret;
    }
    printk("  ✓ Enable pin 0 (P0.%d) configured\n", ENABLE_PIN_0);
    
    // Configure enable pin 1 (P0.6)
    ret = gpio_pin_configure(gpio_device, ENABLE_PIN_1, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("❌ ERROR: Failed to configure enable pin 1\n");
        return ret;
    }
    printk("  ✓ Enable pin 1 (P0.%d) configured\n", ENABLE_PIN_1);
    
    printk("✓ Enable pins ready\n");
    return 0;
}

/**
 * Initialize PWM device
 * Checks that the PWM hardware is available and ready
 */
static int initialize_pwm(void)
{
    printk("\n⚡ Initializing PWM controller...\n");
    
    if (!device_is_ready(pwm_device)) {
        printk("❌ ERROR: PWM device not ready\n");
        return -1;
    }
    
    printk("  ✓ PWM device ready\n");
    printk("  ✓ PWM frequency: %d Hz\n", PWM_FREQUENCY_HZ);
    printk("  ✓ PWM period: %d µs\n", PWM_PERIOD_USEC);
    printk("  ✓ Number of channels: %d\n", NUM_PWM_CHANNELS);
    
    // Turn off all PWM channels initially
    for (int ch = 0; ch < NUM_PWM_CHANNELS; ch++) {
        pwm_set(pwm_device, ch, PWM_USEC(PWM_PERIOD_USEC), 0, 0);
    }
    printk("  ✓ All PWM channels set to OFF\n");
    
    printk("✓ PWM initialized\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
 * TIME MULTIPLEXING - The core magic happens here!
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Set the enable pins to select which group of solenoids is active
 * 
 * This is the "switching" part of time multiplexing. By changing these
 * two pins, we select which of the 4 positions in each PWM group is active.
 * 
 * @param group_number Which group to activate (0, 1, 2, or 3)
 * 
 * Example:
 *   group 0 (binary 00): EN0=0, EN1=0 → Activates solenoids 0, 4, 8, 12
 *   group 1 (binary 01): EN0=1, EN1=0 → Activates solenoids 1, 5, 9, 13
 *   group 2 (binary 10): EN0=0, EN1=1 → Activates solenoids 2, 6, 10, 14
 *   group 3 (binary 11): EN0=1, EN1=1 → Activates solenoids 3, 7, 11, 15
 */
static void set_enable_pins_for_group(uint8_t group_number)
{
    // Extract the two bits that make up the group number
    bool enable_0_state = (group_number & 0x01) != 0;  // Bit 0
    bool enable_1_state = (group_number & 0x02) != 0;  // Bit 1
    
    // Set the physical pins
    gpio_pin_set(gpio_device, ENABLE_PIN_0, enable_0_state ? 1 : 0);
    gpio_pin_set(gpio_device, ENABLE_PIN_1, enable_1_state ? 1 : 0);
}

/**
 * Update PWM outputs for all channels based on current multiplex group
 *
 * This function looks at which solenoids should be active in the current
 * group, and sets the PWM outputs accordingly.
 *
 * Implements "Kick + Hold" logic:
 * - Initial KICK_DURATION_MS: Full power for strong strike
 * - After kick: Reduced power (HOLD_POWER_PERCENT) for efficient holding
 * - Auto-release: Automatically turns off solenoids after MAX_SOLENOID_ON_TIME_MS
 *
 * @param group_number Current group being activated (0-3)
 */
static void update_pwm_for_current_group(uint8_t group_number)
{
    // Go through each PWM channel
    for (int pwm_ch = 0; pwm_ch < NUM_PWM_CHANNELS; pwm_ch++) {
        // Calculate which solenoid this PWM channel + group represents
        int solenoid_num = (pwm_ch * SOLENOIDS_PER_PWM_CHANNEL) + group_number;
        
        // Is this solenoid supposed to be active?
        if (solenoids[solenoid_num].is_active) {
            // Increment cycle counter for this solenoid
            solenoids[solenoid_num].cycles_active++;
            
            #if ENABLE_AUTO_RELEASE
            // SAFETY: Auto-release if solenoid has been on too long
            // Convert cycles to approximate milliseconds for safety check
            uint32_t time_active_ms = solenoids[solenoid_num].cycles_active * MULTIPLEX_PERIOD_MS;
            if (time_active_ms >= MAX_SOLENOID_ON_TIME_MS) {
                printk("⚠️  AUTO-RELEASE: Solenoid %2d (%s) turned off after %d cycles (%d ms)\n",
                       solenoid_num,
                       get_note_name(solenoid_num),
                       solenoids[solenoid_num].cycles_active,
                       time_active_ms);
                solenoids[solenoid_num].is_active = false;
                solenoids[solenoid_num].velocity = 0;
                solenoids[solenoid_num].cycles_active = 0;
                // Fall through to normal PWM off logic below
            }
            #endif
            
            // Re-check active state (may have been cleared by auto-release)
            if (!solenoids[solenoid_num].is_active) {
                // Solenoid should be off
                uint32_t period = PWM_USEC(PWM_PERIOD_USEC);
                pwm_set(pwm_device, pwm_ch, period, 0, 0);
                continue;
            }
            
            // Determine velocity based on kick vs hold phase (cycle-based)
            // KICK: Always use FULL POWER (127) for reliable activation
            // HOLD: Use requested velocity scaled by HOLD_POWER_PERCENT
            uint8_t effective_velocity;
            
            if (solenoids[solenoid_num].cycles_active <= KICK_CYCLES) {
                // KICK PHASE: Use FULL POWER (127) for reliable activation
                effective_velocity = 127;
                solenoids[solenoid_num].in_kick_phase = true;
            } else {
                // HOLD PHASE: Use requested velocity scaled by hold percentage
                effective_velocity = (solenoids[solenoid_num].velocity * HOLD_POWER_PERCENT) / 100;
                solenoids[solenoid_num].in_kick_phase = false;
            }
            
            // Set PWM to the calculated power level
            uint32_t period = PWM_USEC(PWM_PERIOD_USEC);
            uint32_t pulse = velocity_to_pwm_pulse(effective_velocity);
            pwm_set(pwm_device, pwm_ch, period, pulse, 0);
        } else {
            // No, this solenoid should be off
            uint32_t period = PWM_USEC(PWM_PERIOD_USEC);
            pwm_set(pwm_device, pwm_ch, period, 0, 0);
        }
    }
}

/**
 * Main time multiplexing function - called periodically by timer
 * 
 * This is the heart of the time multiplexing system. It:
 * 1. Moves to the next group
 * 2. Sets enable pins to activate that group
 * 3. Updates PWM outputs for solenoids in that group
 */
static void do_time_multiplex_cycle(void)
{
    // Move to next group (cycles through 0 → 1 → 2 → 3 → 0 → ...)
    current_multiplex_group = (current_multiplex_group + 1) % SOLENOIDS_PER_PWM_CHANNEL;
    
    // Set enable pins to select this group
    set_enable_pins_for_group(current_multiplex_group);
    
    // Update PWM outputs for solenoids in this group
    update_pwm_for_current_group(current_multiplex_group);
}

/* ═══════════════════════════════════════════════════════════════════
 * USER-FRIENDLY CONTROL FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Turn on a solenoid at a specific power level
 * 
 * This is the main function you'll use to control solenoids!
 * Just tell it which solenoid and how hard to hit.
 * 
 * @param solenoid_number Which solenoid to activate (0-15)
 * @param velocity Power level (0=off, 127=maximum, like MIDI)
 */
void activate_solenoid(uint8_t solenoid_number, uint8_t velocity)
{
    if (solenoid_number >= TOTAL_SOLENOIDS) {
        printk("⚠️  WARNING: Solenoid %d doesn't exist (valid range: 0-%d)\n",
               solenoid_number, TOTAL_SOLENOIDS - 1);
        return;
    }
    
    // Update solenoid state
    solenoids[solenoid_number].velocity = velocity;
    solenoids[solenoid_number].is_active = (velocity > 0);
    
    // Initialize cycle counter for kick+hold logic
    if (velocity > 0) {
        solenoids[solenoid_number].cycles_active = 0;
        solenoids[solenoid_number].in_kick_phase = true;
    } else {
        solenoids[solenoid_number].cycles_active = 0;
    }
    
    // Calculate power percentage for display
    int power_percent = (velocity * 100) / 127;
    
    if (velocity > 0) {
        printk("🎹 Solenoid %2d (%s) ON  - Power: %3d%% (velocity %d) [KICK+HOLD]\n",
               solenoid_number,
               get_note_name(solenoid_number),
               power_percent,
               velocity);
    } else {
        printk("🎹 Solenoid %2d (%s) OFF\n",
               solenoid_number,
               get_note_name(solenoid_number));
    }
}

/**
 * Turn off a solenoid
 * 
 * @param solenoid_number Which solenoid to turn off (0-15)
 */
void deactivate_solenoid(uint8_t solenoid_number)
{
    activate_solenoid(solenoid_number, 0);
}

/**
 * Turn off ALL solenoids at once
 * Useful for emergency stop or reset
 */
void deactivate_all_solenoids(void)
{
    printk("🛑 Turning off ALL solenoids\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
    }
}

/**
 * Strike a solenoid briefly (like pressing a piano key)
 * 
 * @param solenoid_number Which solenoid (0-15)
 * @param velocity How hard to strike (0-127)
 * @param duration_ms How long to keep it on (milliseconds)
 */
void strike_solenoid(uint8_t solenoid_number, uint8_t velocity, int duration_ms)
{
    activate_solenoid(solenoid_number, velocity);
    k_msleep(duration_ms);
    deactivate_solenoid(solenoid_number);
}

/* ═══════════════════════════════════════════════════════════════════
 * HARDWARE TEST SEQUENCES
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Run a series of tests to verify all solenoids work
 */
static void run_comprehensive_hardware_test(void)
{
    printk("\n");
    printk("╔════════════════════════════════════════════════════════╗\n");
    printk("║         HARDWARE TEST - 16 Solenoid System           ║\n");
    printk("╚════════════════════════════════════════════════════════╝\n");
    printk("\n");
    
    // Test 1: Fire each solenoid individually
    printk("🧪 Test 1: Individual solenoid test\n");
    printk("   (Each solenoid fires once at 50%% power)\n\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        printk("   ");
        strike_solenoid(i, 64, 100);  // 50% power for 100ms
        k_msleep(200);
    }
    k_msleep(1000);
    
    // Test 2: Test each PWM group
    printk("\n🧪 Test 2: PWM group test\n");
    printk("   (Fire all solenoids in each PWM group)\n\n");
    for (int group = 0; group < NUM_PWM_CHANNELS; group++) {
        printk("   Group %d (PWM Channel %d):\n", group, group);
        for (int pos = 0; pos < SOLENOIDS_PER_PWM_CHANNEL; pos++) {
            int solenoid = (group * SOLENOIDS_PER_PWM_CHANNEL) + pos;
            printk("     ");
            strike_solenoid(solenoid, 80, 100);
            k_msleep(150);
        }
        k_msleep(500);
    }
    k_msleep(1000);
    
    // Test 3: Power ramp
    printk("\n🧪 Test 3: Power level test\n");
    printk("   (Solenoid 0 at increasing power levels)\n\n");
    uint8_t power_levels[] = {32, 64, 96, 127};  // 25%, 50%, 75%, 100%
    for (int i = 0; i < 4; i++) {
        printk("   ");
        strike_solenoid(0, power_levels[i], 150);
        k_msleep(300);
    }
    k_msleep(1000);
    
    // Test 4: Simultaneous activation (up to 4 at once)
    printk("\n🧪 Test 4: Simultaneous solenoid test\n");
    printk("   (4 solenoids active at same time - one from each PWM channel)\n\n");
    printk("   Activating solenoids 0, 4, 8, 12 together...\n");
    activate_solenoid(0, 100);
    activate_solenoid(4, 100);
    activate_solenoid(8, 100);
    activate_solenoid(12, 100);
    k_msleep(300);
    deactivate_all_solenoids();
    printk("   All OFF\n");
    k_msleep(1000);
    
    // Test 5: Cascading pattern
    printk("\n🧪 Test 5: Wave pattern\n");
    printk("   (Cascading effect across all solenoids)\n\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        activate_solenoid(i, 100);
        k_msleep(80);
    }
    k_msleep(200);
    for (int i = TOTAL_SOLENOIDS - 1; i >= 0; i--) {
        deactivate_solenoid(i);
        k_msleep(80);
    }
    k_msleep(1000);
    
    printk("\n✓ All tests complete!\n");
    printk("══════════════════════════════════════════════════════════\n\n");
}

/* ═══════════════════════════════════════════════════════════════════
 * UART COMMUNICATION (Optional)
 * ═══════════════════════════════════════════════════════════════════ */

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

void parse_uart_message(char *msg)
{
    int note, velocity;
    if (sscanf(msg, "%d:%d", &note, &velocity) == 2) {
        int solenoid = note - BASE_MIDI_NOTE;
        if (solenoid >= 0 && solenoid < TOTAL_SOLENOIDS) {
            activate_solenoid(solenoid, velocity);
        }
    }
}
#endif

/* ═══════════════════════════════════════════════════════════════════
 * TIMER - Drives the time multiplexing
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * Timer callback - runs periodically to cycle through multiplex groups
 */
void multiplex_timer_handler(struct k_timer *timer)
{
    do_time_multiplex_cycle();
}

// Define the timer
K_TIMER_DEFINE(multiplex_timer, multiplex_timer_handler, NULL);

/* ═══════════════════════════════════════════════════════════════════
 * MAIN PROGRAM
 * ═══════════════════════════════════════════════════════════════════ */

int main(void)
{
    // Print welcome banner
    printk("\n\n");
    printk("╔════════════════════════════════════════════════════════╗\n");
    printk("║                                                        ║\n");
    printk("║          KLAVIATOR - 16 Solenoid Controller           ║\n");
    printk("║           Time Multiplexing System v1.0                ║\n");
    printk("║                                                        ║\n");
    printk("╚════════════════════════════════════════════════════════╝\n");
    printk("\n");
    
    // Display system configuration
    printk("📊 System Configuration:\n");
    printk("   Total solenoids: %d\n", TOTAL_SOLENOIDS);
    printk("   PWM channels: %d\n", NUM_PWM_CHANNELS);
    printk("   Solenoids per channel: %d\n", SOLENOIDS_PER_PWM_CHANNEL);
    printk("   PWM frequency: %d Hz\n", PWM_FREQUENCY_HZ);
    printk("   Multiplex frequency: %d Hz\n", MULTIPLEX_FREQUENCY_HZ);
    printk("   Enable pins: P0.%d, P0.%d\n", ENABLE_PIN_0, ENABLE_PIN_1);
    printk("   Mode: %s\n", ENABLE_UART_MODE ? "UART Control" : "Hardware Test");
    
    // Initialize all subsystems
    printk("\n🚀 Starting initialization...\n");
    
    if (initialize_pwm() < 0) {
        printk("\n❌ FATAL: PWM initialization failed!\n");
        return -1;
    }
    
    if (initialize_enable_pins() < 0) {
        printk("\n❌ FATAL: GPIO initialization failed!\n");
        return -1;
    }
    
    initialize_solenoid_data();
    
    printk("\n✅ All systems initialized successfully!\n");
    
    // Start the time multiplexing timer
    printk("\n⏱️  Starting time multiplexing timer...\n");
    printk("   Switching between groups every %d ms\n", MULTIPLEX_PERIOD_MS);
    k_timer_start(&multiplex_timer, K_MSEC(MULTIPLEX_PERIOD_MS), K_MSEC(MULTIPLEX_PERIOD_MS));
    printk("✓ Timer running\n");
    
    printk("\n");
    printk("╔════════════════════════════════════════════════════════╗\n");
    printk("║              SYSTEM READY FOR OPERATION                ║\n");
    printk("╚════════════════════════════════════════════════════════╝\n");
    printk("\n");
    
    k_msleep(2000);  // Give user time to read
    
    #if ENABLE_UART_MODE
    // UART control mode
    printk("📡 UART Control Mode Active\n");
    printk("   Listening for commands...\n");
    printk("   Format: NOTE:VELOCITY\n");
    printk("   Example: 60:100 (Middle C at 79%% power)\n\n");
    
    if (!device_is_ready(uart_dev)) {
        printk("❌ ERROR: UART not ready\n");
        return -1;
    }
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    
    char msg_buf[MSG_SIZE];
    while (1) {
        if (k_msgq_get(&uart_msgq, &msg_buf, K_FOREVER) == 0) {
            parse_uart_message(msg_buf);
        }
    }
    
    #else
    // Hardware test mode
    printk("🔧 Hardware Test Mode Active\n");
    printk("   Running automated tests...\n\n");
    
    int test_cycle = 1;
    while (1) {
        printk("════════════════════════════════════════════════════════\n");
        printk("                    Test Cycle #%d\n", test_cycle);
        printk("════════════════════════════════════════════════════════\n");
        
        run_comprehensive_hardware_test();
        
        test_cycle++;
        printk("⏸️  Waiting 5 seconds before next cycle...\n");
        printk("   (Reset board to stop)\n\n");
        k_msleep(5000);
    }
    #endif
    
    return 0;
}
