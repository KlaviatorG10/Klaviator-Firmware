# PCA9685 Test Guide

## Hardware Connections

Your PCA9685 should be connected as follows:

```
nRF54L15 Board  →  PCA9685 Module
─────────────────────────────────
P1.07           →  SCL (with 4.7kΩ pullup to 3.3V)
P1.08           →  SDA (with 4.7kΩ pullup to 3.3V)
VCC (3.3V)      →  VCC
GND             →  GND, OE (output enable tied to ground)
```

### Important Notes:
- **Pull-up resistors**: I2C requires 4.7kΩ pull-up resistors on both SCL and SDA lines to 3.3V
- **OE pin**: The Output Enable pin should be tied to GND for the PCA9685 to output
- **Address pins**: A0-A5 should be tied to GND for address 0x40

## Configuration Summary

The firmware has been configured for PCA9685 testing:

1. **I2C Enabled** in [`prj.conf`](prj.conf:10)
2. **I2C1 configured** in [`app.overlay`](app.overlay:16-36) with P1.07=SCL, P1.08=SDA
3. **TEST_MODE set to 0** in [`src/main.c`](src/main.c:53) to run PCA9685 test instead of motor test

## Build and Flash

### Option 1: Using your build system
```bash
# Build the project (exact command depends on your setup)
# Then flash using the flash.sh script:
./flash.sh
```

### Option 2: If using nRF Connect SDK directly
```bash
# Navigate to project directory
cd c:/Users/early/Documents/GitHub/Klaviator-Firmware

# Build
west build -b nrf54l15dk/nrf54l15/cpuapp -p

# Flash
west flash
```

## Expected Output

When the firmware runs, you should see output like:

```
╔══════════════════════════════════════════════════════╗
║  KLAVIATOR V4.0                                     ║
║  MODE: PCA9685 SOLENOID TEST                        ║
╚══════════════════════════════════════════════════════╝

[INIT] PCA9685 on I2C1 addr=0x40 prescaler=0x1D (~200Hz)
[INIT] PCA9685 ready
[INIT] Solenoid map (PCA9685 CH0-15):
  Sol  0  CH 0  Note  60 (C)
  Sol  1  CH 1  Note  61 (C#)
  ...
```

## Test Sequence

The firmware will automatically run a test sequence:

1. **Phase 1 (T=0.5-2.0s)**: Velocity sweep on solenoid 0 (vel 30/64/127)
2. **Phase 2 (T=3.0s)**: 500ms hold to watch KICK→HOLD transition
3. **Phase 3 (T=4.0s)**: Rapid fire x4
4. **Phase 4 (T=5.5s)**: All 16 channels ascending
5. **Phase 5 (T=9.0s)**: Second pass softer
6. **Phase 6 (T=12.5s)**: STOP

## Troubleshooting

### If you see "I2C1 not ready"

This could mean:
1. **Wrong I2C instance**: The nRF54L15 might use `i2c0` or a different instance
   - Edit [`app.overlay`](app.overlay:33) to try `&i2c0` instead of `&i2c1`
   - Edit [`src/main.c`](src/main.c:91) to use `DT_NODELABEL(i2c0)`

2. **Missing pull-ups**: I2C requires pull-up resistors (4.7kΩ recommended)

3. **Hardware not connected**: Double-check wiring

### Checking I2C Communication

If the PCA9685 initializes but doesn't seem to work:
1. The test will pulse channel 0 first at various velocities
2. Check if you see `[KICK]`, `[HOLD]`, and `[RELEASE]` messages
3. If using an LED on a PCA9685 output, it should light up/dim according to the test

### Alternative: Use i2c0

If `i2c1` doesn't compile, try changing to `i2c0`:

**In [`app.overlay`](app.overlay:16-36):**
```dts
&pinctrl {
    i2c0_default: i2c0_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 1, 8)>,
                    <NRF_PSEL(TWIM_SCL, 1, 7)>;
        };
    };

    i2c0_sleep: i2c0_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 1, 8)>,
                    <NRF_PSEL(TWIM_SCL, 1, 7)>;
            low-power-enable;
        };
    };
};

&i2c0 {
    compatible = "nordic,nrf-twim";
    status = "okay";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";
    clock-frequency = <I2C_BITRATE_STANDARD>;
};
```

**In [`src/main.c`](src/main.c:91):**
```c
#define I2C_DEV_NODE            DT_NODELABEL(i2c0)
```

## Next Steps After Successful Test

Once the PCA9685 is working:
1. Connect a solenoid driver circuit (e.g., MOSFET + solenoid)
2. Test individual solenoid control
3. Switch back to motor mode (`TEST_MODE=1`) to test the stepper motor
4. Enable UART mode (`ENABLE_UART_MODE=1`) for dashboard control

## Questions?

- **What is I2C?** Inter-Integrated Circuit - a 2-wire communication protocol
- **Why pull-ups?** I2C uses open-drain outputs and needs pull-ups to work
- **What's the address 0x40?** The default I2C address of the PCA9685 chip
