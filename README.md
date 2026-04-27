# Klaviator Firmware

Production-ready solenoid controller for the Klaviator V4.0 robotic piano.

## 🎯 Current Status

✅ **Production Firmware:** [`src/main.c`](src/main.c) - Solenoid controller  
✅ **Hardware Tested:** All 16 channels working on LEDs  
✅ **Ready For:** Real solenoid deployment

## 📁 Project Structure

```
Klaviator-Firmware/
├── src/
│   ├── main.c                ⭐ PRODUCTION SOLENOID CONTROLLER
│   └── test_led_basic.c      🔧 LED wiring test (troubleshooting)
│
├── CMakeLists.txt            Build configuration
├── prj.conf                  Zephyr config
├── app.overlay               Device tree (I2C, GPIO, UART)
│
└── Documentation/
    ├── TESTING_WORKFLOW.md   Complete testing guide
    ├── SOLENOID_TEST_GUIDE.md Hardware setup
    └── PROJECT_STRUCTURE.md   File organization
```

## 🎹 Features

### Production Firmware (main.c)

- **16-Channel Control:** PCA9685 PWM driver
- **Kick-Hold System:** 20ms full power kick → velocity-scaled hold
- **Velocity Mapping:** MIDI velocity (0-127) to PWM with quadratic curve
- **Auto-Release:** Configurable duration per strike
- **Test Sequence:** 7 comprehensive test phases
- **ISR-Safe:** I2C operations properly handled in thread context

## 🔌 Hardware Requirements

### PCA9685 Connections
```
nRF54L15          PCA9685
----------------------------------
P1.07 (SCL)  →    SCL (+ 4.7kΩ pullup to 3.3V)
P1.08 (SDA)  →    SDA (+ 4.7kΩ pullup to 3.3V)
3.3V         →    VCC
GND          →    GND, OE, A0-A5
```

### Solenoid Driver (per channel)
```
PCA9685 CHx  →  100Ω  →  MOSFET gate (IRLZ44N)
MOSFET source  →  GND
MOSFET drain   →  Solenoid negative
Solenoid positive  →  +12V power supply
1N4007 diode across solenoid (cathode to +12V)
```

## 🚀 Quick Start

### 1. Build
```bash
# Using nRF Connect for VS Code extension:
Click "Build [pristine]"

# Or command line:
west build -b nrf54l15dk/nrf54l15/cpuapp --pristine
```

### 2. Flash
```bash
# Using nRF Connect:
Click "Flash"

# Or command line:
west flash
```

### 3. Monitor
```bash
# Using nRF Connect:
Click "Serial Monitor"

# Or command line:
west serial
```

## 🧪 Test Sequence

The firmware runs a comprehensive test on boot:

1. **Velocity Sweep** - Soft, medium, loud strikes
2. **Kick-Hold Demo** - 500ms sustained note
3. **Rapid Fire** - 5 quick pulses  
4. **Chromatic Scale** - All 16 channels sequentially
5. **C Major Chord** - 3 simultaneous strikes
6. **C Minor Chord** - 3 simultaneous strikes
7. **Power Test** - All 16 channels at once

### Expected Output
```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - SOLENOID CONTROLLER                    ║
║  Fresh Build - Solenoids Only                       ║
╚══════════════════════════════════════════════════════╝

[INIT] PCA9685 ready (~203Hz PWM)
[INIT] Solenoid mapping (16 channels)
[READY] Starting test sequence...

[KICK]    Sol  0  CH 0    C  vel= 40  dur= 100ms  T=2704
[HOLD]    Sol  0  CH 0    C  vel= 40  pwm=1064   T=2724
[AUTO-REL] Sol  0  CH 0    C  T=2804
...
```

## ⚙️ Configuration

Edit [`src/main.c`](src/main.c) to tune behavior:

```c
#define TOTAL_SOLENOIDS     16      // Number of solenoids (1-16)
#define KICK_DURATION_MS    20      // Full power kick (10-50ms)
#define MIN_PWM_PERCENT     18      // Hold power minimum (10-30%)
#define TIMER_PERIOD_MS     5       // Control loop speed (5-10ms)
```

## 📚 Documentation

- **[TESTING_WORKFLOW.md](TESTING_WORKFLOW.md)** - Complete testing guide from LEDs to solenoids
- **[SOLENOID_TEST_GUIDE.md](SOLENOID_TEST_GUIDE.md)** - Hardware wiring and tuning
- **[PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)** - File organization
- **[FLASH_INSTRUCTIONS.md](FLASH_INSTRUCTIONS.md)** - Build and flash guide

## 🔧 Troubleshooting

### No LEDs/Solenoids Work

1. Check serial output for I2C errors
2. Verify PCA9685 connections (SCL, SDA, VCC, GND)
3. Confirm 4.7kΩ pullup resistors on I2C lines
4. Use `test_led_basic.c` to verify hardware

### Some Channels Don't Work

- Check individual MOSFET connections
- Verify channel mapping (CH0-CH15)
- Test with LEDs first before solenoids

### Weak Strikes

- Increase `KICK_DURATION_MS` (try 30ms)
- Check 12V power supply under load
- Verify MOSFET is logic-level (IRLZ44N)

## 🛠️ Development

### Switch to LED Test
Edit [`CMakeLists.txt`](CMakeLists.txt) line 35:
```cmake
target_sources(app PRIVATE
    src/test_led_basic.c    # Switch to LED test
)
```

Always use **Build [pristine]** after switching!

### Git Workflow
```bash
git add -A
git commit -m "Your message"
git push origin main
```

## 📊 System Specifications

- **Platform:** nRF54L15DK (nrf54l15dk/nrf54l15/cpuapp)
- **SDK:** nRF Connect SDK v2.9.2
- **Zephyr:** v3.7.99
- **I2C:** 100kHz standard mode
- **PWM:** ~203Hz (PCA9685)
- **Flash:** 34,256 bytes
- **RAM:** 5,552 bytes

## 🎵 MIDI Note Mapping

```
Channel  MIDI  Note    Channel  MIDI  Note
----------------------------------------------
CH0      60    C       CH8      68    G#
CH1      61    C#      CH9      69    A
CH2      62    D       CH10     70    A#
CH3      63    D#      CH11     71    B
CH4      64    E       CH12     72    C'
CH5      65    F       CH13     73    C#'
CH6      66    F#      CH14     74    D'
CH7      67    G       CH15     75    D#'
```

## 🚦 Project Status

- ✅ **Phase 1:** LED wiring verified
- ✅ **Phase 2:** Solenoid patterns tested on LEDs
- 🔜 **Phase 3:** Real solenoid deployment
- 🔜 **Phase 4:** UART integration for dashboard control
- 🔜 **Phase 5:** Motor control integration

## 📝 License

See project license file.

## 👨‍💻 Contributors

Klaviator V4.0 Development Team

---

**Last Updated:** 2026-04-27  
**Version:** 1.0.0  
**Status:** Production Ready for Solenoid Testing
