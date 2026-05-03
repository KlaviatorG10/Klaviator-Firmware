# How to Flash LED Test to Microcontroller

## Important: Only ONE main file is compiled at a time!

The build system only compiles the file specified in `src/CMakeLists.txt`. Other main files in the `src/` folder are ignored and won't interfere.

Current configuration:
- ✅ `main_led_test.c` - **ACTIVE** (will be flashed)
- ⚪ `main.c` - ignored during build
- ⚪ `main_simple_test.c` - ignored during build

## Step-by-Step Flashing

### 1. Connect Hardware
- Connect nRF54L15 DK to computer via USB
- Wire PCA9685 to the board:
  ```
  P1.07 → PCA9685 SCL (+ 4.7kΩ pullup to 3.3V)
  P1.08 → PCA9685 SDA (+ 4.7kΩ pullup to 3.3V)
  3.3V  → PCA9685 VCC
  GND   → PCA9685 GND, OE, A0-A5
  ```
- Connect LEDs to PCA9685 channels:
  ```
  CH0-15 → 330Ω resistor → LED anode (+) → LED cathode (-) → GND
  ```

### 2. Build Firmware
Open terminal in project directory:
```bash
cd c:/Users/early/Documents/GitHub/Klaviator-Firmware
```

Clean build (recommended for switching between main files):
```bash
west build -b nrf54l15dk/nrf54l15/cpuapp --pristine
```

Or using nRF Connect CLI:
```bash
nrf-connect-sdk-cli build --pristine
```

### 3. Flash to Microcontroller
```bash
west flash
```

Or:
```bash
nrf-connect-sdk-cli flash
```

### 4. Monitor Serial Output
```bash
west serial
```

Or:
```bash
nrf-connect-sdk-cli serial
```

Press `Ctrl+C` to exit serial monitor.

## Expected Output

After flashing, the serial monitor should show:

```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - PCA9685 LED TEST                       ║
╚══════════════════════════════════════════════════════╝

[INIT] Initializing PCA9685...
[INIT] I2C: i2c21, Address: 0x40
[INIT] PCA9685 initialized successfully!
[INIT] PWM frequency: ~203 Hz

[READY] Starting LED test sequence...

═══════════════════════════════════════════════════════
 TEST PASS #0
═══════════════════════════════════════════════════════

[TEST] Turning ALL LEDs ON...
[TEST] Turning ALL LEDs OFF...
[TEST] Sequential LED test (CH0-CH15)...
[TEST] CH0 ON
[TEST] CH1 ON
...
```

And the LEDs should:
1. All turn ON for 2 seconds
2. All turn OFF for 1 second
3. Blink one by one from CH0 to CH15
4. CH0 shows brightness levels (25%, 50%, 75%, 100%)
5. Wave pattern (3 LEDs chase around)
6. Repeat...

## Switch to Different Main File

To test a different firmware:

1. **Edit `src/CMakeLists.txt`** - change line 13:
   ```cmake
   # For full solenoid/motor firmware:
   target_sources(app PRIVATE main.c)
   
   # OR for LED test:
   target_sources(app PRIVATE main_led_test.c)
   
   # OR for simple test:
   target_sources(app PRIVATE main_simple_test.c)
   ```

2. **Rebuild with --pristine:**
   ```bash
   west build --pristine
   west flash
   ```

**Note:** Always use `--pristine` when switching between main files to ensure a clean build.

## Troubleshooting

### Build Errors

**Error: "cannot find main_led_test.c"**
- Check file exists: `src/main_led_test.c`
- Verify CMakeLists.txt path is correct (no `src/` prefix needed)

**Error: "multiple definition of main"**
- Only one `target_sources()` line should be active
- Comment out other `target_sources()` lines with `#`

### Flash Errors

**Error: "No connected boards"**
- Check USB connection
- Try different USB port
- Check Device Manager (Windows) for nRF54L15 device

**Error: "Flash failed"**
- Press RESET button on board
- Try flashing again
- Power cycle the board

### Hardware Issues

**No LEDs light up:**
1. Check serial output for I2C errors
2. Verify PCA9685 connections (SCL, SDA, VCC, GND)
3. Check pullup resistors (4.7kΩ on SCL and SDA)
4. Verify LED polarity (long leg = anode/+)
5. Test with multimeter: measure 3.3V on PCA9685 VCC pin

**Some LEDs don't work:**
- Check individual LED polarity
- Test LED separately with 3V battery
- Verify resistor value (220Ω-1kΩ)

**I2C errors in console:**
```
[ERROR] i2c21 device not ready!
```
- Missing pullup resistors on SCL/SDA
- Check I2C wiring
- Verify PCA9685 address (0x40 with all address pins to GND)

## Files Reference

- **`src/main_led_test.c`** - LED test firmware source code
- **`src/main.c`** - Full solenoid/motor/UART firmware
- **`src/main_simple_test.c`** - Simple test firmware
- **`src/CMakeLists.txt`** - Build configuration (select which main to compile)
- **`PCA9685_LED_TEST_GUIDE.md`** - Detailed hardware guide
- **`app.overlay`** - Device tree (I2C, GPIO, UART configuration)
- **`prj.conf`** - Zephyr project configuration
