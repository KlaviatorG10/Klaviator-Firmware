/*
 * =============================================================================
 * KLAVIATOR - SIMPLE PCA9685 LED TEST
 * =============================================================================
 *
 * This is a basic test to verify PCA9685 communication and LED control.
 * LEDs should blink in sequence through channels 0-15.
 *
 * Hardware Setup:
 *   PCA9685:
 *     P1.07  ->  PCA9685 SCL  (+ 4.7k pullup to 3.3V)
 *     P1.08  ->  PCA9685 SDA  (+ 4.7k pullup to 3.3V)
 *     3.3V   ->  PCA9685 VCC
 *     GND    ->  PCA9685 GND, OE, A0-A5
 *
 *   LEDs (each channel):
 *     PCA9685 CH0-15 -> 330Ω resistor -> LED anode (+)
 *     LED cathode (-) -> GND
 *
 * Test Sequence:
 *   1. All LEDs on for 2 seconds
 *   2. All LEDs off for 1 second
 *   3. Individual LEDs blink in sequence (CH0 to CH15)
 *   4. Repeat
 * =============================================================================
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

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
#define PCA9685_PRESCALE_VALUE  0x1D    /* ~203 Hz */

static const struct device *i2c_device = DEVICE_DT_GET(I2C_DEV_NODE);

/* =============================================================================
 * PCA9685 LOW-LEVEL FUNCTIONS
 * ============================================================================= */

static int pca9685_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    int ret = i2c_write(i2c_device, buf, 2, PCA9685_I2C_ADDR);
    if (ret) {
        printk("[ERROR] Write reg 0x%02X = 0x%02X failed: %d\n", reg, val, ret);
    }
    return ret;
}

static int pca9685_set_channel(uint8_t ch, uint16_t value)
{
    if (ch >= 16) {
        printk("[ERROR] Invalid channel %d (must be 0-15)\n", ch);
        return -EINVAL;
    }

    uint8_t reg = PCA9685_LED0_ON_L + (ch * 4);
    uint8_t buf[5];
    buf[0] = reg;

    if (value == 0) {
        /* Full OFF */
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = 0x00;
        buf[4] = 0x10;
    } else if (value >= PCA9685_MAX_PWM) {
        /* Full ON */
        buf[1] = 0x00;
        buf[2] = 0x10;
        buf[3] = 0x00;
        buf[4] = 0x00;
    } else {
        /* PWM value */
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = (uint8_t)(value & 0xFF);
        buf[4] = (uint8_t)(value >> 8);
    }

    int ret = i2c_write(i2c_device, buf, 5, PCA9685_I2C_ADDR);
    if (ret) {
        printk("[ERROR] Set CH%d failed: %d\n", ch, ret);
    }
    return ret;
}

static int pca9685_all_on(void)
{
    int r = 0;
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_L,  0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_ON_H,  0x10);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_L, 0x00);
    r |= pca9685_write_reg(PCA9685_ALL_LED_OFF_H, 0x00);
    return r;
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

/* =============================================================================
 * PCA9685 INITIALIZATION
 * ============================================================================= */

static int init_pca9685(void)
{
    printk("\n[INIT] Initializing PCA9685...\n");
    printk("[INIT] I2C: i2c21, Address: 0x%02X\n", PCA9685_I2C_ADDR);

    if (!device_is_ready(i2c_device)) {
        printk("[ERROR] i2c21 device not ready!\n");
        printk("[ERROR] Check connections:\n");
        printk("        P1.07 -> SCL (+ 4.7k pullup to 3.3V)\n");
        printk("        P1.08 -> SDA (+ 4.7k pullup to 3.3V)\n");
        printk("        VCC   -> 3.3V\n");
        printk("        GND   -> GND\n");
        return -ENODEV;
    }

    /* Put to sleep before changing prescaler */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_SLEEP)) {
        printk("[ERROR] Failed to put PCA9685 to sleep\n");
        return -EIO;
    }
    k_usleep(500);

    /* Set prescaler for ~200Hz PWM frequency */
    if (pca9685_write_reg(PCA9685_PRESCALE, PCA9685_PRESCALE_VALUE)) {
        printk("[ERROR] Failed to set prescaler\n");
        return -EIO;
    }

    /* Wake up and enable auto-increment */
    if (pca9685_write_reg(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL)) {
        printk("[ERROR] Failed to wake up PCA9685\n");
        return -EIO;
    }
    k_usleep(500);

    /* Set MODE2 for totem pole output */
    if (pca9685_write_reg(PCA9685_MODE2, 0x04)) {
        printk("[ERROR] Failed to set MODE2\n");
        return -EIO;
    }

    /* Turn all channels off initially */
    if (pca9685_all_off()) {
        printk("[ERROR] Failed to turn off all channels\n");
        return -EIO;
    }

    printk("[INIT] PCA9685 initialized successfully!\n");
    printk("[INIT] PWM frequency: ~203 Hz\n");
    return 0;
}

/* =============================================================================
 * TEST FUNCTIONS
 * ============================================================================= */

static void test_all_leds_on(void)
{
    printk("\n[TEST] Turning ALL LEDs ON...\n");
    pca9685_all_on();
    k_msleep(2000);
}

static void test_all_leds_off(void)
{
    printk("[TEST] Turning ALL LEDs OFF...\n");
    pca9685_all_off();
    k_msleep(1000);
}

static void test_sequential_blink(void)
{
    printk("[TEST] Sequential LED test (CH0-CH15)...\n");
    
    for (int ch = 0; ch < 16; ch++) {
        printk("[TEST] CH%d ON\n", ch);
        pca9685_set_channel(ch, PCA9685_MAX_PWM);
        k_msleep(200);
        
        pca9685_set_channel(ch, 0);
        k_msleep(100);
    }
}

static void test_brightness_levels(void)
{
    printk("\n[TEST] Brightness test on CH0 (25%%, 50%%, 75%%, 100%%)...\n");
    
    uint16_t levels[] = {
        PCA9685_MAX_PWM / 4,   // 25%
        PCA9685_MAX_PWM / 2,   // 50%
        PCA9685_MAX_PWM * 3 / 4,  // 75%
        PCA9685_MAX_PWM        // 100%
    };
    
    for (int i = 0; i < 4; i++) {
        printk("[TEST] CH0 brightness: %d%% (PWM=%d)\n", 
               (i + 1) * 25, levels[i]);
        pca9685_set_channel(0, levels[i]);
        k_msleep(1000);
    }
    
    pca9685_set_channel(0, 0);
}

static void test_wave_pattern(void)
{
    printk("\n[TEST] Wave pattern (left to right)...\n");
    
    for (int start = 0; start < 16; start++) {
        /* Turn on 3 consecutive LEDs */
        for (int i = 0; i < 3; i++) {
            int ch = (start + i) % 16;
            pca9685_set_channel(ch, PCA9685_MAX_PWM);
        }
        
        k_msleep(150);
        
        /* Turn them off */
        for (int i = 0; i < 3; i++) {
            int ch = (start + i) % 16;
            pca9685_set_channel(ch, 0);
        }
    }
}

/* =============================================================================
 * MAIN
 * ============================================================================= */

int main(void)
{
    printk("\n");
    printk("╔══════════════════════════════════════════════════════╗\n");
    printk("║  KLAVIATOR - PCA9685 LED TEST                       ║\n");
    printk("╚══════════════════════════════════════════════════════╝\n");

    /* Initialize PCA9685 */
    if (init_pca9685() < 0) {
        printk("\n[FATAL] PCA9685 initialization failed!\n");
        printk("[FATAL] Cannot proceed with test.\n");
        while (1) {
            k_msleep(1000);
        }
    }

    printk("\n[READY] Starting LED test sequence...\n");
    printk("[INFO]  Connect LEDs to CH0-CH15:\n");
    printk("        PCA9685 CHx -> 330Ω -> LED+ -> LED- -> GND\n\n");

    k_msleep(2000);

    uint32_t pass = 0;
    
    /* Main test loop */
    while (1) {
        printk("\n");
        printk("═══════════════════════════════════════════════════════\n");
        printk(" TEST PASS #%u\n", pass++);
        printk("═══════════════════════════════════════════════════════\n");

        /* Test 1: All LEDs on */
        test_all_leds_on();

        /* Test 2: All LEDs off */
        test_all_leds_off();

        /* Test 3: Sequential blink */
        test_sequential_blink();

        /* Test 4: Brightness levels */
        test_brightness_levels();

        /* Test 5: Wave pattern */
        test_wave_pattern();

        /* Pause before next pass */
        printk("\n[INFO] Pass complete. Next pass in 3 seconds...\n");
        k_msleep(3000);
    }

    return 0;
}
