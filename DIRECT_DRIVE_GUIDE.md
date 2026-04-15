# KLAVIATOR V4.0 - Direct Drive Guide

## Overview

**1:1 Direct Control with Advanced Features:**
- Each solenoid has its own PWM pin (P1.0-P1.15)
- **Kick-to-Hold transition** - 100% kick for 20ms → smooth hold
- **Duration-based control** - Auto-release after specified time
- **Clock synchronization** - Precise timing for sequencer integration
- **KDAA V4.0 protocol** - S:H:N:V:T:D format

---

## Pin Mapping

| Solenoid | nRF Pin | MIDI Note | Note Name |
|----------|---------|-----------|-----------|
| 0  | P1.0  | 60 | C  |
| 1  | P1.1  | 61 | C# |
| 2  | P1.2  | 62 | D  |
| 3  | P1.3  | 63 | D# |
| 4  | P1.4  | 64 | E  |
| 5  | P1.5  | 65 | F  |
| 6  | P1.6  | 66 | F# |
| 7  | P1.7  | 67 | G  |
| 8  | P1.8  | 68 | G# |
| 9  | P1.9  | 69 | A  |
| 10 | P1.10 | 70 | A# |
| 11 | P1.11 | 71 | B  |
| 12 | P1.12 | 72 | C' |
| 13 | P1.15 | 73 | C#'|
| 14 | P1.16 | 74 | D' |
| 15 | P1.17 | 75 | D#'|

**Note:** P1.13 and P1.14 are reserved for UART (serial communication)

---

## V4.0 Features

### 1. Kick-to-Hold Transition

**What it does:**
- Starts with 100% PWM (the "kick") for 20ms to quickly engage solenoid
- Automatically transitions to velocity-based PWM (the "hold") to maintain position
- Reduces power consumption and heat while maintaining actuation

**Serial output:**
```
[KICK] Sol: 0 | Pin:P1.00 | Vel:100 | Dur: 200ms | T:  1234
[HOLD] Sol: 0 | Pin:P1.00 | Duty: 79% | T:  1254
[RELEASE] Sol: 0 | Pin:P1.00 | T:  1434
```

### 2. Duration-Based Control

**Auto-release after specified duration:**
- Set duration in milliseconds
- Solenoid automatically releases when time elapses
- Perfect for rhythmic sequences

**Example:**
```
1:0:60:100:0:200
```
→ Strike note 60 at velocity 100 for 200ms, then auto-release

### 3. Clock Synchronization

**Precise timing for sequencer integration:**
- `sync` command resets internal clock to T=0
- All events timestamped with synchronized time
- Enables frame-accurate playback from Python Dashboard

**Usage:**
```
sync          → Reset clock
1:0:60:100:0:200  → Events use synchronized time
```

---

## KDAA V4.0 Protocol

### Full Format: S:H:N:V:T:D

**Parameters:**
- **S** = Sequence number (for tracking, currently ignored)
- **H** = Hand/group (0=left, 1=right, currently ignored)
- **N** = MIDI Note (60-75 for solenoids 0-15)
- **V** = Velocity (0-127, 0=off)
- **T** = Target time in ms (for future sequencer scheduling)
- **D** = Duration in ms (0=manual control)

### Examples:

**Basic strike with 200ms duration:**
```
1:0:60:100:0:200
```
- Seq 1, Hand 0 (left)
- Note 60 (solenoid 0)
- Velocity 100 (~79% PWM)
- Time 0ms (immediate)
- Duration 200ms

**Full power, 500ms duration:**
```
2:1:64:127:0:500
```
- Seq 2, Hand 1 (right)
- Note 64 (solenoid 4)
- Velocity 127 (100% PWM during hold)
- Duration 500ms

**Manual control (no auto-release):**
```
3:0:60:80:0:0
```
- Duration 0 = stays on until explicitly released

**Release command:**
```
4:0:60:0:0:0
```
- Velocity 0 = release immediately

---

## Simple Commands (Backward Compatible)

### Format: NOTE:VELOCITY

**Examples:**
```
60:100    → Strike solenoid 0 at velocity 100 (manual control)
60:0      → Release solenoid 0
64:127    → Strike solenoid 4 at full power (manual control)
```

**Note:** Simple format doesn't use duration - you must manually release with `NOTE:0`

---

## Special Commands

```
sync      → Reset clock to T=0 (for sequencer sync)
all:0     → Emergency stop - turn off all solenoids immediately
```

---

## Timing & Control

### Control Timer
- Runs every **5ms** for precise control
- Monitors kick-to-hold transitions
- Checks duration-based auto-release

### Kick Phase
- Duration: **20ms** (defined by `KICK_DURATION_MS`)
- PWM: **100%** duty cycle
- Purpose: Quick solenoid engagement

### Hold Phase
- PWM: Velocity-based (18-100%)
- Duration: Until release or duration expires
- Purpose: Maintain position with less power

---

## Serial Monitor Output

### Startup:
```
╔═══════════════════════════════════════════════════════╗
║  KLAVIATOR V4.0 - Direct Drive Controller            ║
║  Features: Kick-Hold, Duration, Clock Sync           ║
╚═══════════════════════════════════════════════════════╝

[INIT] Initializing PWM controller (1000 Hz)...
[INIT] PWM initialized at 1000 Hz on P1.0-P1.15
  Solenoid  0: Pin P1.00, Note  60 (C)
  Solenoid  1: Pin P1.01, Note  61 (C#)
  ...
[SYNC] Clock synchronized. T=0
[INIT] Starting control timer (5ms period)...
[INIT] Kick duration: 20ms
[READY] System ready
```

### During Operation:
```
[KICK] Sol: 0 | Pin:P1.00 | Vel:100 | Dur: 200ms | T:    123
[HOLD] Sol: 0 | Pin:P1.00 | Duty: 79% | T:    143
[AUTO-RELEASE] Sol: 0 | Duration: 200ms elapsed | T:    323
```

### Errors:
```
[ERROR] Note 50 out of range (valid: 60-75)
[ERROR] Velocity must be 0-127
[ERROR] Invalid command format
```

---

## Wiring

### For Testing with 16 LEDs:
```
P1.0  → 330Ω → LED0 (+) → LED0 (-) → GND
P1.1  → 330Ω → LED1 (+) → LED1 (-) → GND
P1.2  → 330Ω → LED2 (+) → LED2 (-) → GND
...
P1.17 → 330Ω → LED15(+) → LED15(-) → GND
```

### Connections Required:
- 1× GND wire (common ground)
- 16× PWM wires (one per solenoid/LED)
- **Total: 17 wires**

---

## Testing Sequence

### 1. Basic Strike Test
```
sync          → Reset clock
60:100        → Strike solenoid 0 (manual)
```
Expected output:
```
[KICK] Sol: 0 | Pin:P1.00 | Vel:100 | Dur:   0ms | T:      0
[HOLD] Sol: 0 | Pin:P1.00 | Duty: 79% | T:     20
```

### 2. Duration Test
```
sync
1:0:60:100:0:200
```
Expected output:
```
[KICK] Sol: 0 | Pin:P1.00 | Vel:100 | Dur: 200ms | T:      0
[HOLD] Sol: 0 | Pin:P1.00 | Duty: 79% | T:     20
[AUTO-RELEASE] Sol: 0 | Duration: 200ms elapsed | T:    200
```

### 3. Multiple Solenoids
```
sync
1:0:60:100:0:100
2:0:64:80:0:100
3:0:67:120:0:100
```
→ Three solenoids play together, auto-release after 100ms

### 4. Emergency Stop
```
all:0
```
→ Immediately releases all active solenoids

---

## Troubleshooting

### No [KICK] messages:
- Check UART baud rate (must be 1000000)
- Verify command format: `S:H:N:V:T:D` or `NOTE:VELOCITY`
- Check note range: 60-75

### [HOLD] never appears:
- Control timer not running
- Check if firmware built correctly
- Should appear 20ms after [KICK]

### Auto-release not working:
- Check duration parameter (must be > 0)
- Verify control timer is running every 5ms
- Duration=0 means manual control (no auto-release)

### Clock not synchronized:
- Send `sync` command before sequence
- Check serial output for `[SYNC] Clock synchronized. T=0`
- All timestamps should start from 0

### LED doesn't light:
- Verify pin connection (solenoid 0 = P1.0, etc.)
- Check GND connection
- Test with full power: `60:127`
- Check resistor (330Ω, not 330kΩ)

---

## Python Dashboard Integration

### Send sequence:
```python
# Sync clock
serial.write(b"sync\n")

# Send notes with precise timing
serial.write(b"1:0:60:100:0:200\n")  # Note 60, 200ms
time.sleep(0.05)  # 50ms delay
serial.write(b"2:0:64:80:0:150\n")   # Note 64, 150ms
```

### Parse feedback:
```python
while True:
    line = serial.readline().decode()
    if "[KICK]" in line:
        # Extract solenoid, velocity, duration, timestamp
    elif "[HOLD]" in line:
        # Extract solenoid, duty cycle, timestamp
    elif "[RELEASE]" in line:
        # Extract solenoid, timestamp
```

---

## Configuration

### Adjust timing in firmware:

```c
#define KICK_DURATION_MS    20      // Kick phase duration
#define TIMER_PERIOD_MS     5       // Control timer interval
#define MIN_PWM_PERCENT     18      // Minimum PWM for solenoids
```

### To change kick duration:
1. Edit `KICK_DURATION_MS` in `main_direct.c`
2. Rebuild firmware
3. Flash to board

---

## V4.0 vs Earlier Versions

| Feature | V1-3 | V4.0 |
|---------|------|------|
| Kick-to-hold | ❌ No | ✅ Yes (20ms kick) |
| Duration control | ❌ No | ✅ Yes (auto-release) |
| Clock sync | ❌ No | ✅ Yes (`sync` command) |
| Protocol | Simple | KDAA V4.0 (S:H:N:V:T:D) |
| Timestamps | ❌ No | ✅ Yes (synchronized) |
| Distinct logging | Basic | [KICK], [HOLD], [RELEASE] |

---

## Summary

**V4.0 gives you:**
- ✅ Efficient power control (kick-to-hold)
- ✅ Automatic note duration management
- ✅ Frame-accurate timing (clock sync)
- ✅ Rich serial feedback for debugging
- ✅ Ready for Python Dashboard integration

**Perfect for musical performance and precise robotic control!** 🎹🤖
