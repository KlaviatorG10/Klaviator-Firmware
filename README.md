# Klaviator Firmware

## Building
To build the firmware, you can use the Nordic nRF Connect extension in VS Code or run the following command if you have the toolchain installed:
```bash
west build -b nrf54l15dk/nrf54l15/cpuapp --sysbuild --build-dir KDAA
```

## Flashing
If `west flash` is not working (e.g., due to missing `nrfutil`), you can use the provided helper script:
```bash
./flash.sh
```
This script uses `nrfjprog` to flash the `KDAA/merged.hex` file to the first connected J-Link probe.

