# Klaviator Testing Workflow

This document explains the testing workflow from LED verification to real solenoid testing.

## Testing Phases

### Phase 1: LED Wiring Verification ✅
**File:** `src/main_led_test.c`

**Purpose:** Confirm PCA9685 I2C communication and basic channel control.

**What it does:**
- Simple on/off patterns
- Sequential blinking
- Brightness levels
- Wave patterns

**Use this to verify:**
- ✅ I2C is working (P1.07 SCL, P1.08 SDA)
- ✅ PCA9685 is responding
- ✅ All 16 channels output correctly
- ✅ PWM control works

**Build:**
```cmake
# In CMakeLists.txt:
target_sources(app PRIVATE src/main_led_test.c)
```

---

### Phase 2: Solenoid Pattern Testing on LEDs ← **YOU ARE HERE**
**File:** `src/main.c`

**Purpose:** Test solenoid-specific timing and patterns visually on LEDs before connecting real solenoids.

**What it does:**
- **Kick-Hold phases** - Watch LED brightness change from full to dimmed after 20ms
- **Chromatic scale** - All 16 LEDs fire in sequence
- **Chords** - Multiple LEDs simultaneously
- **All solenoids** - All 16 LEDs at once (power test)
- **Velocity mapping** - Different LED brightness levels

**Use this to verify:**
- ✅ Kick phase (full brightness) lasts 20ms
- ✅ Hold phase (dimmer) activates correctly
- ✅ Auto-release timing works
- ✅ Multiple channels can activate together
- ✅ Velocity-to-PWM mapping is smooth

**Build:**
```cmake
# In CMakeLists.txt:
target_sources(app PRIVATE src/main.c)
```

**Test on LEDs:**
```
PCA9685 CH0-15 → 330Ω → LED+ → LED- → GND
```

**Watch for:**
- LEDs should flash bright (kick) then dim slightly (hold)
- Chromatic scale should light each LED in sequence
- Chords should light multiple LEDs together
- All-at-once test should light all 16 LEDs

---

### Phase 3: Real Solenoid Testing
**File:** Same `src/main.c` (no code changes needed!)

**Purpose:** After LED verification, connect real solenoids and run same tests.

**Hardware change:**
```
PCA9685 CH0-15 → 100Ω → MOSFET gate (IRLZ44N)
MOSFET drain → Solenoid-
Solenoid+ → +12V
Flyback diode (1N4007) across solenoid
```

**Power supply:**
- 12V 3-5A for solenoids
- Keep nRF power separate (3.3V USB)
- Common ground between supplies

**Safety checks:**
- ✅ Flyback diodes installed (critical!)
- ✅ MOSFET orientation correct (source to GND)
- ✅ Power supply adequate current
- ✅ No loose connections

---

### Phase 4: UART Integration (Future)
**File:** Add UART code to `src/main.c` or create new version

**Purpose:** Control solenoids from computer/dashboard.

**Not included yet - focus on getting solenoids working first!**

---

## Quick Reference: Switching Between Tests

### Build LED Wiring Test
```cmake
# CMakeLists.txt line 24:
target_sources(app PRIVATE src/main_led_test.c)
```
Then: Build [pristine] → Flash → Serial Monitor

### Build Solenoid Pattern Test  
```cmake
# CMakeLists.txt line 24:
target_sources(app PRIVATE src/main.c)
```
Then: Build [pristine] → Flash → Serial Monitor

---

## Current Test Sequence (main.c)

When you flash `main.c`, it runs this sequence automatically:

```
[Phase 1] Velocity Sweep
  - Sol 0: Soft (40), Medium (80), Loud (127)
  - See LED brightness change

[Phase 2] Kick-Hold Demo
  - Sol 1: 500ms hold
  - Watch for KICK (bright) → HOLD (dimmer) after 20ms

[Phase 3] Rapid Fire
  - Sol 2: 5 rapid strikes
  - Tests timing precision

[Phase 4] Chromatic Scale
  - All 16 solenoids in sequence
  - 200ms each
  - Like playing a scale

[Phase 5] C Major Chord
  - Sol 0, 4, 7 simultaneously
  - Tests parallel control

[Phase 6] C Minor Chord
  - Sol 0, 3, 7 simultaneously
  - Another chord test

[Phase 7] All Solenoids
  - All 16 at once
  - Ultimate power test
```

---

## Console Output Reference

**LED Test (main_led_test.c):**
```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - PCA9685 LED TEST                       ║
╚══════════════════════════════════════════════════════╝

[TEST] Turning ALL LEDs ON...
[TEST] Turning ALL LEDs OFF...
[TEST] Sequential LED test (CH0-CH15)...
```

**Solenoid Pattern Test (main.c):**
```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR - SOLENOID CONTROLLER                    ║
║  Fresh Build - Solenoids Only                       ║
╚══════════════════════════════════════════════════════╝

[KICK]    Sol  0  CH 0  C    vel= 40  dur= 100ms  T=1000
[HOLD]    Sol  0  CH 0  C    vel= 40  pwm= 416  T=1020
[RELEASE] Sol  0  CH 0  C    T=1100
```

---

## Files Overview

```
src/
├── main_led_test.c       ← Phase 1: Basic LED wiring test
├── main.c                ← Phase 2/3: Solenoid patterns (test on LEDs, then solenoids)
└── main_simple_test.c    ← (unused)

CMakeLists.txt            ← Switch between tests here (line 24)
TESTING_WORKFLOW.md       ← This file
SOLENOID_TEST_GUIDE.md    ← Detailed solenoid hardware guide
PCA9685_LED_TEST_GUIDE.md ← LED test guide
FLASH_INSTRUCTIONS.md     ← Build/flash instructions
```

---

## What to Look For During LED Testing

### Kick-Hold Phase (Phase 2)
- **LED should flash bright** (kick) for ~20ms
- **Then dim slightly** (hold) for remainder
- This simulates strong solenoid strike then reduced hold current

### Velocity Levels (Phase 1)
- **Soft (40)**: LED noticeably dimmer
- **Medium (80)**: LED medium brightness
- **Loud (127)**: LED full brightness
- Brightness should scale smoothly

### Timing Precision (Phase 3)
- **Rapid fire**: LEDs should blink cleanly with no overlap
- Each strike should be distinct

### Simultaneous Control (Phase 5-7)
- **Chords**: Multiple LEDs light together
- **All at once**: All 16 LEDs should light
- Check if any LEDs flicker (power issue)

---

## Troubleshooting

### LEDs don't show kick-hold transition
- Transition happens in 20ms (very fast)
- You might need to slow it down to see clearly
- Edit `KICK_DURATION_MS` in main.c to 100ms for testing

### Some LEDs don't light during "all solenoids" test
- Power supply may be insufficient
- Normal with USB power + many LEDs
- Should work better with 12V supply for real solenoids

### Timing seems off
- Check `TIMER_PERIOD_MS` is 5ms
- Verify control timer is running (check console)

---

## Next Steps

1. ✅ **Phase 1 Complete** - LED wiring verified with `main_led_test.c`
2. **→ Phase 2 Current** - Test solenoid patterns on LEDs with `main.c`
3. **Phase 3 Next** - Connect real solenoids (same code, different hardware)
4. **Phase 4 Future** - Add UART control for dashboard integration
