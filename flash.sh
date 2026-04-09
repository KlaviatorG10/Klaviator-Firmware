#!/bin/zsh

# Helper script to flash Klaviator Firmware using nrfjprog

BUILD_DIR="KDAA"
HEX_FILE="${BUILD_DIR}/merged.hex"

if [ ! -f "$HEX_FILE" ]; then
    echo "Error: ${HEX_FILE} not found. Did you build the project?"
    exit 1
fi

# Get the first connected J-Link probe ID
BOARD_ID=$(nrfjprog --ids | head -n 1)

if [ -z "$BOARD_ID" ] || [ "$BOARD_ID" = "" ]; then
    echo "Error: No connected boards found."
    exit 1
fi

echo "Flashing ${HEX_FILE} to board ${BOARD_ID}..."

nrfjprog --program "$HEX_FILE" --chiperase --verify --reset --snr "$BOARD_ID"

if [ $? -eq 0 ]; then
    echo "Flash successful!"
else
    echo "Flash failed."
    exit 1
fi
