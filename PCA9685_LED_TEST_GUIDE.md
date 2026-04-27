# PCA9685 LED Test Guide

This guide shows how to test the PCA9685 PWM driver with simple LEDs using the `main_led_test.c` firmware.

## Hardware Setup

### PCA9685 Connections
```
nRF54L15           PCA9685
----------------------------------------
P1.07 (SCL)   ->   SCL (+ 4.7kΩ to 3.3V)
P1.08 (SDA)   ->   SDA (+ 4.7kΩ to 3.3V)
3.3V          ->   VCC
GND           ->   GND, OE, A0-A5
```

### LED Connections (Per Channel)
```
PCA9685 CH0-15  ->  330Ω resistor  ->  LED anode (+, longer leg)
                    LED cathode (-, shorter leg)  ->  GND
```

**Notes:**
- Use 330Ω-1kΩ resistors to limit current
- Standard 5mm LEDs work well
- Longer LED leg is anode (+), shorter is cathode (-)
- For brighter LEDs, use lower resistance (not below 220Ω)

### Alternative: Using MOSFETs (For Brighter LEDs)
```
PCA9685 CHx  ->  10kΩ  ->  MOSFET gate (2N7000, BS170, etc.)
MOSFET drain  ->  LED cathode
LED anode  ->  +5V/+12V (through appropriate resistor)
MOSFET source  ->  GND
```

## Build and Flash

### 1. Update CMakeLists.txt
Edit `src/CMakeLists.txt` to use the LED test:
```cmake
target_sources(app PRIVATE main_led_test.c)
```

### 2. Build
```bash
cd c:/Users/early/Documents/GitHub/Klaviator-Firmware
nrf-connect-sdk-cli build
```

### 3. Flash
```bash
nrf-connect-sdk-cli flash
```

### 4. Monitor Output
```bash
nrf-connect-sdk-cli serial
```

## Test Sequence

The test runs continuously in a loop with 5 phases:

### Phase 1: All LEDs ON (2 seconds)
- All 16 channels turn on simultaneously
- Verifies bulk control and power supply

### Phase 2: All LEDs OFF (1 second)
- All channels turn off
- Clears the display

### Phase 3: Sequential Blink (0.3s per channel)
- Each channel (CH0-CH15) blinks individually
- Helps identify channel mapping
- ON for 200ms, OFF for 100ms

### Phase 4: Brightness Test on CH0 (4 seconds)
- Channel 0 displays 4 brightness levels:
  - 25% brightness (1s)
  - 50% brightness (1s)
  - 75% brightness (1s)
  - 100% brightness (1s)
- Verifies PWM control

### Phase 5: Wave Pattern (2.4 seconds)
- 3 consecutive LEDs sweep from CH0 to CH15
- Creates a "chasing" light effect
- Repeats around the channels

Then loop repeats from Phase 1.

## Expected Output

```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - PCA9685 LED TEST                       ║
╚══════════════════════════════════════════════════════╝

[INIT] Initializing PCA9685...
[INIT] I2C: i2c21, Address: 0x40
[INIT] PCA9685 initialized successfully!
[INIT] PWM frequency: ~203 Hz

[READY] Starting LED test sequence...
[INFO]  Connect LEDs to CH0-CH15:
        PCA9685 CHx -> 330Ω -> LED+ -> LED- -> GND

═══════════════════════════════════════════════════════
 TEST PASS #0
═══════════════════════════════════════════════════════

[TEST] Turning ALL LEDs ON...
[TEST] Turning ALL LEDs OFF...
[TEST] Sequential LED test (CH0-CH15)...
[TEST] CH0 ON
[TEST] CH1 ON
...
[TEST] CH15 ON

[TEST] Brightness test on CH0 (25%, 50%, 75%, 100%)...
[TEST] CH0 brightness: 25% (PWM=1023)
[TEST] CH0 brightness: 50% (PWM=2047)
[TEST] CH0 brightness: 75% (PWM=3071)
[TEST] CH0 brightness: 100% (PWM=4095)

[TEST] Wave pattern (left to right)...

[INFO] Pass complete. Next pass in 3 seconds...
```

## Troubleshooting

### No LEDs Light Up
1. **Check I2C connections:**
   - Verify SCL on P1.07 and SDA on P1.08
   - Ensure 4.7kΩ pullup resistors on both lines to 3.3V
   - Check VCC = 3.3V and GND connections

2. **Check console output:**
   ```
   [ERROR] i2c21 device not ready!
   ```
   → I2C hardware not configured properly

3. **Verify PCA9685 address:**
   - Default is 0x40 (all address pins to GND)
   - Run I2C scan to detect: `nrf-connect-sdk-cli i2c-scan`

### Some LEDs Don't Work
- **Check LED polarity:** Long leg = anode (+), short leg = cathode (-)
- **Check resistor value:** Should be 220Ω-1kΩ
- **Test LED separately:** Apply 3V through resistor to verify LED works

### LEDs Very Dim
- **Lower resistor value:** Try 220Ω instead of 330Ω or 1kΩ
- **Check power supply:** PCA9685 needs clean 3.3V supply
- **Use MOSFET driver:** For brighter LEDs with external power

### I2C Errors in Console
```
[ERROR] Write reg 0x00 = 0x10 failed: -5
```
- **Check pullup resistors:** Must have 4.7kΩ on SCL and SDA
- **Check wire length:** Keep I2C wires short (<30cm)
- **Check connections:** Ensure solid connections, no loose wires

## Return to Main Firmware

To switch back to the main firmware:

Edit `src/CMakeLists.txt`:
```cmake
target_sources(app PRIVATE main.c)
```

Then rebuild and flash.

## Next Steps

After verifying basic LED control:
1. Test with solenoid drivers (IRLZ44N MOSFETs)
2. Connect stepper motor (see `DIRECT_DRIVE_GUIDE.md`)
3. Enable UART mode for dashboard control (see `SERIAL_MONITOR_GUIDE.md`)
