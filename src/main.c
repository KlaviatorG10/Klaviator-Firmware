/*
 * =============================================================================
 * KLAVIATOR - PCA9685 SOLENOID CONTROLLER (Clean Build)
 * =============================================================================
 *
 * Fresh firmware focused purely on solenoid control via PCA9685.
 * No motor, no UART - just clean solenoid testing.
 *
 * Hardware Setup:
 *   PCA9685:
 *     P1.07  ->  PCA9685 SCL  (+ 4.7k pullup to 3.3V)
 *     P1.08  ->  PCA9685 SDA  (+ 4.7k pullup to 3.3V)
 *     3.3V   ->  PCA9685 VCC
 *     GND    ->  PCA9685 GND, OE, A0-A5
 *
 *   Solenoids (per channel):
 *     PCA9685 CH0-15 -> 100Ω -> IRLZ44N gate
 *     IRLZ44N source -> GND
 *     IRLZ44N drain  -> Solenoid negative
 *     Solenoid positive -> 12V
 *     1N4007 diode across solenoid (cathode to +12V)
 *
 * Test Sequence:
 *   - Individual solenoid strikes (CH0-15)
 *   - Velocity control demonstration
 *   - Kick-hold phase demonstration
 *   - Chord testing (multiple solenoids)
 * =============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* =============================================================================
 * CONFIGURATION
 * ============================================================================= */

#define TOTAL_SOLENOIDS     16
#define BASE_MIDI_NOTE      60          /* Middle C (C4) */
#define KICK_DURATION_MS    20          /* Full power kick phase */
#define MIN_PWM_PERCENT     18          /* Minimum hold power */
#define TIMER_PERIOD_MS     5           /* Control loop frequency */

/* PCA9685 Configuration */
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
 * PCA9685 DRIVER
 * ============================================================================= */

static int pca9685_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write(i2c_device, buf, 2, PCA9685_I2C_ADDR);
    if (ret) {
        printk("[ERROR] PCA9685 write reg=0x%02X val=0x%02X err=%d\n", reg, val, ret);
    }
    /* Small delay to allow I2C bus to settle */
    k_usleep(100);
    return ret;
}

static int pca9685_set_channel(uint8_t ch, uint16_t value)
{
    if (ch >= 16) {
        return -EINVAL;
    }

    uint8_t reg_base = PCA9685_LED0_ON_L + (ch * 4);
    int ret = 0;

    if (value == 0) {
        /* Full OFF - set bit 4 of OFF_H register */
        ret |= pca9685_write_reg(reg_base + 0, 0x00);  /* ON_L */
        ret |= pca9685_write_reg(reg_base + 1, 0x00);  /* ON_H */
        ret |= pca9685_write_reg(reg_base + 2, 0x00);  /* OFF_L */
        ret |= pca9685_write_reg(reg_base + 3, 0x10);  /* OFF_H = bit 4 set */
    } else if (value >= PCA9685_MAX_PWM) {
        /* Full ON - set bit 4 of ON_H register */
        ret |= pca9685_write_reg(reg_base + 0, 0x00);  /* ON_L */
        ret |= pca9685_write_reg(reg_base + 1, 0x10);  /* ON_H = bit 4 set */
        ret |= pca9685_write_reg(reg_base + 2, 0x00);  /* OFF_L */
        ret |= pca9685_write_reg(reg_base + 3, 0x00);  /* OFF_H */
    } else {
        /* PWM value - set ON=0, OFF=value */
        ret |= pca9685_write_reg(reg_base + 0, 0x00);              /* ON_L = 0 */
        ret |= pca9685_write_reg(reg_base + 1, 0x00);              /* ON_H = 0 */
        ret |= pca9685_write_reg(reg_base + 2, (uint8_t)(value & 0xFF));    /* OFF_L */
        ret |= pca9685_write_reg(reg_base + 3, (uint8_t)(value >> 8));      /* OFF_H */
    }

    if (ret) {
        printk("[ERROR] PCA9685 set CH%d=%u err=%d\n", ch, value, ret);
    }

    return ret;
}

static int pca9685_all_off(void)
{
    int r = 0;
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_L,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_H,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x10);
    return r;
}

static int init_pca9685(void)
{
    printk("\n[INIT] PCA9685 on i2c21 addr=0x%02X\n", PCA9685_I2C_ADDR);

    if (!device_is_ready(i2c_device)) {
        printk("[ERROR] i2c21 not ready\n");
        printk("[ERROR] Check: P1.07=SCL P1.08=SDA (4.7k pullups) VCC=3.3V GND=GND\n");
        return -ENODEV;
    }

    /* Sleep mode to set prescaler */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_SLEEP)) {
        return -EIO;
    }
    k_usleep(500);

    /* Set PWM frequency ~203Hz */
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VALUE)) {
        return -EIO;
    }

    /* Wake up with auto-increment */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) {
        return -EIO;
    }
    k_usleep(500);

    /* Totem pole output */
    if (pca9685_write_reg(PCA9685_MODE2, 0x04)) {
        return -EIO;
    }

    /* Turn all off */
    if (pca9685_all_off()) {
        return -EIO;
    }

    printk("[INIT] PCA9685 ready (~203Hz PWM)\n");
    return 0;
}

/* =============================================================================
 * VELOCITY TO PWM CONVERSION (Integer Math)
 * ============================================================================= */

static uint16_t velocity_to_pwm(uint8_t velocity)
{
    if (velocity == 0) {
        return 0;
    }
    if (velocity >= 127) {
        return PCA9685_MAX_PWM;
    }

    /* Quadratic mapping: MIN_PWM_PERCENT to 100% based on velocity^2 */
    uint32_t v2 = (uint32_t)velocity * velocity;
    uint32_t pct = MIN_PWM_PERCENT + (v2 * (100U - MIN_PWM_PERCENT)) / (127U * 127U);
    return (uint16_t)((pct * PCA9685_MAX_PWM) / 100U);
}

/* =============================================================================
 * NOTE NAME HELPER
 * ============================================================================= */

static const char *note_name(uint8_t solenoid_num)
{
    static const char *names[] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
        "C'", "C#'", "D'", "D#'"
    };
    return (solenoid_num < TOTAL_SOLENOIDS) ? names[solenoid_num] : "?";
}

/* =============================================================================
 * SOLENOID CONTROL
 * ============================================================================= */

void activate_solenoid(uint8_t sol, uint8_t velocity, uint32_t duration_ms)
{
    if (sol >= TOTAL_SOLENOIDS) {
        return;
    }

    struct Solenoid *s = &solenoids[sol];

    if (velocity > 0) {
        s->velocity = velocity;
        s->is_active = true;
        s->duration_target_ms = duration_ms;
        s->activation_time = k_uptime_get_32();
        s->kick_start_time = s->activation_time;
        s->in_kick_phase = true;

        /* Full power kick */
        pca9685_set_channel(s->channel, PCA9685_MAX_PWM);
        
        printk("[KICK]    Sol %2d  CH%2d  %3s  vel=%3d  dur=%4dms  T=%u\n",
               sol, s->channel, note_name(sol), velocity, duration_ms, 
               s->activation_time);
    } else {
        /* Release */
        s->velocity = 0;
        s->is_active = false;
        s->in_kick_phase = false;
        s->duration_target_ms = 0;

        pca9685_set_channel(s->channel, 0);
        
        printk("[RELEASE] Sol %2d  CH%2d  %3s  T=%u\n",
               sol, s->channel, note_name(sol), k_uptime_get_32());
    }
}

void deactivate_solenoid(uint8_t sol)
{
    activate_solenoid(sol, 0, 0);
}

void deactivate_all_solenoids(void)
{
    printk("[ALL-OFF] Releasing all solenoids\n");
    pca9685_all_off();

    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].velocity = 0;
        solenoids[i].is_active = false;
        solenoids[i].in_kick_phase = false;
        solenoids[i].duration_target_ms = 0;
    }
}

/* =============================================================================
 * CONTROL TIMER (Kick->Hold + Auto-Release)
 *
 * NOTE: Timer callbacks run in ISR context - we CANNOT do I2C writes here!
 * Instead, we set flags and let the main thread handle actual I2C operations.
 * ============================================================================= */

/* Flags for pending actions */
static volatile bool pending_actions[TOTAL_SOLENOIDS];

static void control_timer_callback(struct k_timer *timer)
{
    uint32_t now = k_uptime_get_32();

    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        struct Solenoid *s = &solenoids[i];
        
        if (!s->is_active) {
            continue;
        }

        /* Check if kick->hold transition needed */
        if (s->in_kick_phase && (now - s->kick_start_time) >= KICK_DURATION_MS) {
            s->in_kick_phase = false;
            pending_actions[i] = true;  /* Signal main thread to update */
        }

        /* Check if auto-release needed */
        if (s->duration_target_ms > 0 &&
            (now - s->activation_time) >= s->duration_target_ms) {
            s->is_active = false;  /* Mark for release */
            pending_actions[i] = true;  /* Signal main thread */
        }
    }
}

K_TIMER_DEFINE(control_timer, control_timer_callback, NULL);

/* Process pending solenoid updates (call from main thread) */
static void process_solenoid_updates(void)
{
    uint32_t now = k_uptime_get_32();
    
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        if (!pending_actions[i]) {
            continue;
        }
        
        pending_actions[i] = false;
        struct Solenoid *s = &solenoids[i];
        
        if (!s->is_active) {
            /* Release solenoid */
            pca9685_set_channel(s->channel, 0);
            printk("[AUTO-REL] Sol %2d  CH%2d  %3s  T=%u\n",
                   i, s->channel, note_name(i), now);
            s->duration_target_ms = 0;
        } else if (!s->in_kick_phase) {
            /* Transition to hold */
            uint16_t hold_pwm = velocity_to_pwm(s->velocity);
            pca9685_set_channel(s->channel, hold_pwm);
            printk("[HOLD]    Sol %2d  CH%2d  %3s  vel=%3d  pwm=%4u  T=%u\n",
                   i, s->channel, note_name(i), s->velocity, hold_pwm, now);
        }
    }
}

/* =============================================================================
 * SOLENOID INITIALIZATION
 * ============================================================================= */

static void init_solenoid_data(void)
{
    printk("\n[INIT] Solenoid mapping (16 channels):\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        solenoids[i].channel = (uint8_t)i;
        solenoids[i].is_active = false;
        solenoids[i].velocity = 0;
        printk("  Sol %2d  ->  CH%2d  MIDI %3d  Note %3s\n",
               i, i, BASE_MIDI_NOTE + i, note_name(i));
    }
    printk("\n");
}

/* =============================================================================
 * TEST SEQUENCE
 * ============================================================================= */

static void run_solenoid_test_sequence(void)
{
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  SOLENOID TEST SEQUENCE                             ║\n");
    printk("╚══════════════════════════════════════════════════════╝\n");

    /* Phase 1: Individual strikes with different velocities */
    printk("\n[TEST] Phase 1: Velocity sweep on Sol 0 (soft -> medium -> loud)\n");
    activate_solenoid(0, 40, 100);   /* Soft */
    k_msleep(500);
    activate_solenoid(0, 80, 100);   /* Medium */
    k_msleep(500);
    activate_solenoid(0, 127, 100);  /* Loud */
    k_msleep(500);

    /* Phase 2: Long hold to observe kick->hold transition */
    printk("\n[TEST] Phase 2: Long hold (500ms) - watch KICK->HOLD transition\n");
    activate_solenoid(1, 90, 500);
    k_msleep(1000);

    /* Phase 3: Rapid fire */
    printk("\n[TEST] Phase 3: Rapid fire x5 on Sol 2\n");
    for (int i = 0; i < 5; i++) {
        activate_solenoid(2, 100, 60);
        k_msleep(80);
    }
    k_msleep(500);

    /* Phase 4: Chromatic scale (all solenoids sequentially) */
    printk("\n[TEST] Phase 4: Chromatic scale (all 16 solenoids)\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        activate_solenoid(i, 85, 120);
        k_msleep(200);
    }
    k_msleep(500);

    /* Phase 5: Chord test (multiple solenoids simultaneously) */
    printk("\n[TEST] Phase 5: C Major chord (Sol 0, 4, 7)\n");
    activate_solenoid(0, 95, 300);  /* C */
    activate_solenoid(4, 95, 300);  /* E */
    activate_solenoid(7, 95, 300);  /* G */
    k_msleep(800);

    printk("\n[TEST] Phase 6: C Minor chord (Sol 0, 3, 7)\n");
    activate_solenoid(0, 95, 300);  /* C */
    activate_solenoid(3, 95, 300);  /* D# */
    activate_solenoid(7, 95, 300);  /* G */
    k_msleep(800);

    /* Phase 6: All solenoids at once (power test) */
    printk("\n[TEST] Phase 7: ALL solenoids simultaneously (power test)\n");
    for (int i = 0; i < TOTAL_SOLENOIDS; i++) {
        activate_solenoid(i, 70, 200);
    }
    k_msleep(500);

    printk("\n[TEST] Sequence complete!\n\n");
}

/* =============================================================================
 * MAIN
 * ============================================================================= */

int main(void)
{
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  KLAVIATOR - SOLENOID CONTROLLER                    ║\n");
    printk("║  Fresh Build - Solenoids Only                       ║\n");
    printk("╚══════════════════════════════════════════════════════╝\n");

    k_msleep(100);

    /* Initialize PCA9685 */
    if (init_pca9685() < 0) {
        printk("\n[FATAL] PCA9685 initialization failed\n");
        printk("[FATAL] Check I2C wiring and pullups\n");
        while (1) {
            k_msleep(1000);
        }
    }

    /* Initialize solenoid data structures */
    init_solenoid_data();

    /* Start control timer for kick->hold transitions */
    k_timer_start(&control_timer, K_MSEC(TIMER_PERIOD_MS), K_MSEC(TIMER_PERIOD_MS));
    printk("[INIT] Control timer started (%dms period)\n", TIMER_PERIOD_MS);
    printk("[INIT] Kick duration: %dms\n", KICK_DURATION_MS);
    printk("[INIT] Hold power: %d%% minimum\n\n", MIN_PWM_PERCENT);

    printk("[READY] System ready\n");
    printk("[READY] Starting test sequence in 2 seconds...\n\n");
    k_msleep(2000);

    /* Run test sequence once */
    run_solenoid_test_sequence();

    /* Main processing loop */
    printk("[INFO] Test complete. Entering main loop.\n");
    printk("[INFO] Processing solenoid updates continuously...\n\n");

    uint32_t heartbeat = 0;
    while (1) {
        /* Process any pending solenoid state changes */
        process_solenoid_updates();
        
        /* Heartbeat every 5 seconds */
        if ((k_uptime_get_32() % 5000) < 10) {
            if (heartbeat % 100 == 0) {  /* Only print occasionally */
                printk("[HB] Alive %u  (processing loop active)\n", heartbeat);
            }
            heartbeat++;
        }
        
        /* Small delay to avoid busy-waiting */
        k_msleep(1);
    }

    return 0;
}
