# Klaviator Firmware - Project Structure

## Active Project Files

### Main Firmware Files

**[`src/main.c`](src/main.c)** - **PRIMARY SOLENOID CONTROLLER** ⭐
- Complete solenoid control with kick-hold phases
- 16-channel PCA9685 PWM control
- Velocity-to-PWM mapping
- Auto-release timing
- Test sequence (velocity/chromatic/chords)
- Ready for real solenoids
- **This is the main project file - currently active**

**[`src/main_led_test.c`](src/main_led_test.c)** - **LED WIRING TEST**
- Simple LED on/off patterns
- Used to verify PCA9685 connections
- Basic I2C communication test
- Switch to this for hardware troubleshooting

### Configuration Files

**[`CMakeLists.txt`](CMakeLists.txt)** - Build configuration
- Line 24 specifies which main file to compile
- Currently set to `src/main.c`
- Change to `src/main_led_test.c` for LED testing

**[`prj.conf`](prj.conf)** - Zephyr project configuration
- Enables GPIO, I2C, UART, Console
- Kernel settings

**[`app.overlay`](app.overlay)** - Device tree overlay
- I2C pin configuration (P1.07=SCL, P1.08=SDA)
- GPIO configuration
- UART configuration

### Documentation

**[`README.md`](README.md)** - Project overview

**[`TESTING_WORKFLOW.md`](TESTING_WORKFLOW.md)** - Step-by-step testing guide
- Phase 1: LED wiring verification
- Phase 2: Solenoid patterns on LEDs (current phase)
- Phase 3: Real solenoid testing

**[`SOLENOID_TEST_GUIDE.md`](SOLENOID_TEST_GUIDE.md)** - Solenoid hardware guide
- Wiring diagrams
- MOSFET connections
- Power supply requirements
- Tuning parameters

**[`PCA9685_LED_TEST_GUIDE.md`](PCA9685_LED_TEST_GUIDE.md)** - LED test hardware guide

**[`FLASH_INSTRUCTIONS.md`](FLASH_INSTRUCTIONS.md)** - Build and flash instructions

**[`SERIAL_MONITOR_GUIDE.md`](SERIAL_MONITOR_GUIDE.md)** - Serial monitor setup

**[`DIRECT_DRIVE_GUIDE.md`](DIRECT_DRIVE_GUIDE.md)** - Motor control guide (future)

**[`PCA9685_TEST_GUIDE.md`](PCA9685_TEST_GUIDE.md)** - PCA9685 general guide

### Reference Files (Not Compiled)

**[`../../../Downloads/main.c`](../../../Downloads/main.c)** - Previous version reference
- Combined motor + solenoid + UART firmware
- Kept for reference
- Not used in current build

**[`../../../Downloads/main_2.c`](../../../Downloads/main_2.c)** - Another reference version
- Kept for reference
- Not used in current build

### Build Artifacts (Auto-generated)

**`build/`** - Build output directory (ignored by git)
**`Klaviator-Firmware`** - Binary output

## File Organization

```
Klaviator-Firmware/
├── src/
│   ├── main.c                    ← MAIN PROJECT FILE (solenoid controller)
│   ├── main_led_test.c           ← LED test (for troubleshooting)
│   └── CMakeLists.txt
│
├── CMakeLists.txt                ← Build config (selects which main)
├── prj.conf                      ← Project configuration
├── app.overlay                   ← Device tree (pin config)
│
├── Documentation/
│   ├── README.md
│   ├── TESTING_WORKFLOW.md       ← START HERE
│   ├── SOLENOID_TEST_GUIDE.md
│   ├── PCA9685_LED_TEST_GUIDE.md
│   ├── FLASH_INSTRUCTIONS.md
│   ├── SERIAL_MONITOR_GUIDE.md
│   ├── DIRECT_DRIVE_GUIDE.md
│   └── PROJECT_STRUCTURE.md      ← This file
│
├── build/                        ← Auto-generated (gitignored)
│
└── Reference/
    └── ../../../Downloads/
        ├── main.c                ← Reference: full firmware
        └── main_2.c              ← Reference: variant
```

## Quick Switch Between Firmwares

### Use Solenoid Controller (Current)
```cmake
# In CMakeLists.txt line 24:
target_sources(app PRIVATE src/main.c)
```

### Use LED Test
```cmake
# In CMakeLists.txt line 24:
target_sources(app PRIVATE src/main_led_test.c)
```

Always use **Build [pristine]** after switching!

## Current Status

✅ **Active Firmware:** `src/main.c` (Solenoid Controller)
✅ **Hardware Status:** LED testing complete, ready for real solenoids
✅ **Test Status:** All 7 phases working perfectly
✅ **I2C Status:** Working reliably (error -5 fixed)
✅ **Next Step:** Connect real solenoids with MOSFETs

## Development History

1. ✅ Initial project setup
2. ✅ PCA9685 I2C configuration
3. ✅ LED test firmware created
4. ✅ LED wiring verified
5. ✅ Solenoid controller firmware created
6. ✅ Fixed I2C interrupt context issue (error -5)
7. ✅ **Current:** Ready for real solenoid testing
8. 🔜 **Next:** UART integration for dashboard control

## Removed Files

~~`src/main_simple_test.c`~~ - Removed (not used)

## Build Commands

```bash
# Build with nRF Connect for VS Code
Click "Build [pristine]" → "Flash" → "Serial Monitor"

# Build with command line
west build -b nrf54l15dk/nrf54l15/cpuapp --pristine
west flash
west serial
```

## Git Status

Use the following to save progress:
```bash
git add -A
git commit -m "Solenoid controller working - LED test complete"
git push origin main
```
