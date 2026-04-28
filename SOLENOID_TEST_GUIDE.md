# Solenoid Testing Guide - Fresh Build

This is a clean, simplified firmware for testing PCA9685 solenoid control without motor or UART complexity.

## Features

- ✅ **16 Solenoid Control** - Full PCA9685 PWM control
- ✅ **Velocity Mapping** - MIDI velocity (0-127) to PWM with quadratic curve
- ✅ **Kick-Hold Phases** - 20ms full power kick, then velocity-scaled hold
- ✅ **Auto-Release** - Configurable duration per strike
- ✅ **Clean Code** - No motor, no UART, easy to understand

## Hardware Setup

### PCA9685 Connections
```
nRF54L15           PCA9685
----------------------------------
P1.07 (SCL)   ->   SCL (+ 4.7kΩ to 3.3V)
P1.08 (SDA)   ->   SDA (+ 4.7kΩ to 3.3V)
3.3V          ->   VCC
GND           ->   GND, OE, A0-A5
```

### Solenoid Driver (per channel)
```
PCA9685 CHx  ->  100Ω resistor  ->  MOSFET gate (IRLZ44N)
MOSFET source  ->  GND
MOSFET drain   ->  Solenoid negative (-)
Solenoid positive (+)  ->  +12V supply
1N4007 diode across solenoid (cathode to +12V, anode to solenoid-)
```

**Important:**
- Use 12V 3A+ power supply for solenoids
- Keep nRF power (3.3V) separate from solenoid power (12V)
- Common ground between nRF and solenoid supply
- Flyback diodes (1N4007) are critical to protect MOSFETs

## Build and Flash

### Switch to Solenoid Firmware

Edit `CMakeLists.txt` line 24:
```cmake
target_sources(app PRIVATE
    src/main.c        # ← Solenoid firmware
)
```

### Build
```bash
# Using nRF Connect for VS Code: Click "Build [pristine]"
```

Or command line:
```bash
west build -b nrf54l15dk/nrf54l15/cpuapp --pristine
```

### Flash
```bash
# Using nRF Connect: Click "Flash"
```

Or command line:
```bash
west flash
```

### Monitor
```bash
# Using nRF Connect: Click "Serial Monitor"
```

Or command line:
```bash
west serial
```

## Test Sequence

The firmware runs a comprehensive test sequence once on boot:

### Phase 1: Velocity Sweep (Sol 0)
- **Soft strike** (vel=40) - 100ms
- **Medium strike** (vel=80) - 100ms  
- **Loud strike** (vel=127) - 100ms

Tests velocity-to-PWM mapping.

### Phase 2: Kick-Hold Transition (Sol 1)
- 500ms sustained note
- Watch console for KICK→HOLD transition after 20ms

### Phase 3: Rapid Fire (Sol 2)
- 5 strikes with 80ms spacing
- Tests response time

### Phase 4: Chromatic Scale (All 16)
- Each solenoid strikes sequentially
- 200ms spacing
- Verifies all channels work

### Phase 5: C Major Chord (Sol 0, 4, 7)
- Simultaneous strike test
- Tests power supply and timing

### Phase 6: C Minor Chord (Sol 0, 3, 7)
- Different chord voicing
- More simultaneous testing

### Phase 7: All Solenoids (Power Test)
- All 16 fire at once
- Ultimate power supply test
- Check for voltage sag

## Expected Console Output

```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - SOLENOID CONTROLLER                    ║
║  Fresh Build - Solenoids Only                       ║
╚══════════════════════════════════════════════════════╝

[INIT] PCA9685 on i2c21 addr=0x40
[INIT] PCA9685 ready (~203Hz PWM)

[INIT] Solenoid mapping (16 channels):
  Sol  0  ->  CH 0  MIDI  60  Note C
  Sol  1  ->  CH 1  MIDI  61  Note C#
  ...
  Sol 15  ->  CH15  MIDI  75  Note D#'

[INIT] Control timer started (5ms period)
[INIT] Kick duration: 20ms
[INIT] Hold power: 18% minimum

[READY] System ready
[READY] Starting test sequence in 2 seconds...

╔══════════════════════════════════════════════════════╗
║  SOLENOID TEST SEQUENCE                             ║
╚══════════════════════════════════════════════════════╝

[TEST] Phase 1: Velocity sweep on Sol 0 (soft -> medium -> loud)
[KICK]    Sol  0  CH 0  C    vel= 40  dur= 100ms  T=1234
[HOLD]    Sol  0  CH 0  C    vel= 40  pwm= 416  T=1254
[RELEASE] Sol  0  CH 0  C    T=1334
[KICK]    Sol  0  CH 0  C    vel= 80  dur= 100ms  T=1834
...
```

## Configuration Options

Edit `src/main.c` to adjust behavior:

```c
#define TOTAL_SOLENOIDS     16      // Number of solenoids (1-16)
#define BASE_MIDI_NOTE      60      // Starting MIDI note (C4)
#define KICK_DURATION_MS    20      // Full power kick phase (10-50ms)
#define MIN_PWM_PERCENT     18      // Minimum hold power (10-30%)
#define TIMER_PERIOD_MS     5       // Control loop speed (5-10ms)
```

### Tuning Recommendations

**KICK_DURATION_MS:**
- Shorter (10-15ms): Faster, lighter strikes
- Longer (20-30ms): Stronger, more powerful strikes
- Too long (>50ms): Wastes power, overheats solenoids

**MIN_PWM_PERCENT:**
- Lower (10-15%): Saves power, but may not hold solenoid
- Higher (20-30%): Secure hold, but more heat
- Test with your specific solenoids

## Troubleshooting

### No solenoids activate
1. **Check I2C:** Serial should show `[INIT] PCA9685 ready`
2. **Check power:** 12V supply connected and adequate current
3. **Check MOSFETs:** Gate voltage should be 0-3.3V PWM
4. **Check diodes:** Flyback diodes installed correctly

### Weak strikes
- Increase `KICK_DURATION_MS` (try 30ms)
- Check 12V power supply voltage under load
- Verify MOSFET is IRLZ44N (logic-level)
- Check wire resistance from PSU to solenoids

### Solenoids stay on / don't release
- Check `control_timer` is running
- Verify `TIMER_PERIOD_MS` is reasonable (5-10ms)
- Look for `[HOLD]` and `[RELEASE]` messages in console

### Some channels don't work
- Check PCA9685 channel mapping (CH0-15)
- Test with LED test firmware first
- Verify MOSFET gate connections
- Check for cold solder joints

### I2C errors
```
[ERROR] i2c21 not ready
```
- Missing pullup resistors (need 4.7kΩ on SCL and SDA)
- Check P1.07 and P1.08 connections
- Verify PCA9685 power (3.3V on VCC pin)

## Next Steps

### Add Manual Control
Modify the main loop to accept button input or simple commands to trigger specific solenoids.

### UART Integration
Once solenoid testing is complete, you can add UART control for the dashboard by enabling the UART code sections.

### Motor Integration  
After solenoids work reliably, add back the stepper motor code for position control.

## Switch Between Firmware Versions

### To LED Test:
```cmake
# In CMakeLists.txt:
target_sources(app PRIVATE src/main_led_test.c)
```

### To Full Firmware (motor + UART):
```cmake
# In CMakeLists.txt:
target_sources(app PRIVATE src/main.c)
# Then manually enable UART and motor features in the code
```

Always use pristine build when switching: `Build [pristine]` in nRF Connect.
