#!/bin/zsh

# Helper script to flash Klaviator Firmware using nrfutil (for nRF54 series)

# Gjør stiene absolutte basert på hvor scriptet ligger
SCRIPT_DIR="${0:A:h}"
BUILD_DIR="${SCRIPT_DIR}/KDAA"
HEX_FILE="${BUILD_DIR}/merged.hex"
NRFUTIL="${SCRIPT_DIR}/nrfutil_mac"

if [ ! -f "$HEX_FILE" ]; then
    echo "Error: ${HEX_FILE} not found. Did you build the project?"
    exit 1
fi

if [ ! -f "$NRFUTIL" ]; then
    echo "Error: nrfutil_mac not found in project root."
    exit 1
fi

# Get the first connected J-Link probe ID safely using nrfutil
BOARD_ID=$("$NRFUTIL" device list | grep -o -E '^[0-9]{9,10}' | head -n 1)

if [ -z "$BOARD_ID" ] || [ "$BOARD_ID" = "" ]; then
    echo "Error: No connected boards found."
    exit 1
fi

echo "Flashing ${HEX_FILE} to board ${BOARD_ID}..."

"$NRFUTIL" device program --firmware "$HEX_FILE" --serial-number "$BOARD_ID"
# Note: nrfutil device program automatically handles erase and reset for nRF54.

if [ $? -eq 0 ]; then
    echo "Flash successful!"
else
    echo "Flash failed."
    exit 1
fi
