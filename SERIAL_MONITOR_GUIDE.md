# How to Open Serial Monitor for Klaviator Debugging

## Quick Method: Using nRF Connect Serial Terminal

1. **In VS Code**, look at the left sidebar
2. Click the **nRF Connect** icon (Nordic Semiconductor logo)
3. Under "CONNECTED DEVICES", find your nRF54L15DK
4. Click **"Serial Terminal"** or **"Open Serial Monitor"**
5. Set baud rate to **1000000** (1 Mbaud - very important!)

## Alternative: Using VS Code Built-in Serial Monitor

1. Press **Ctrl+Shift+P** (or Cmd+Shift+P on Mac)
2. Type: **"Serial Monitor: Focus on Serial Monitor View"**
3. Click **"Start Monitoring"**
4. Configure:
   - **Port**: Select your COM port (e.g., COM3, COM4)
   - **Baud Rate**: **1000000**

## Alternative: Using Windows PowerShell

Open PowerShell and run:
```powershell
# Replace COM3 with your actual COM port
mode COM3:1000000,n,8,1
$port = new-Object System.IO.Ports.SerialPort COM3,1000000,None,8,one
$port.Open()
while($true) {
    if ($port.BytesToRead -gt 0) {
        [System.Console]::Write($port.ReadExisting())
    }
    Start-Sleep -Milliseconds 100
}
```

## Alternative: Using PuTTY (Windows)

1. Download PuTTY if you don't have it
2. Open PuTTY
3. Select **"Serial"** connection type
4. Enter:
   - **Serial line**: COM3 (or your COM port)
   - **Speed**: 1000000
5. Click **"Open"**

## How to Find Your COM Port

### Method 1: Device Manager (Windows)
1. Press **Win + X**, select **"Device Manager"**
2. Expand **"Ports (COM & LPT)"**
3. Look for **"JLink CDC UART Port (COMx)"** or **"USB Serial Device (COMx)"**
4. Note the COM port number

### Method 2: nRF Connect
1. In VS Code, open nRF Connect extension
2. Look under **"CONNECTED DEVICES"**
3. Your board should show the COM port

## What You Should See

If the firmware is running correctly, you should see output like:

```
╔════════════════════════════════════════════════════════╗
║          KLAVIATOR - 16 Solenoid Controller           ║
║           Time Multiplexing System v1.0                ║
╚════════════════════════════════════════════════════════╝

📊 System Configuration:
   Total solenoids: 16
   PWM channels: 4
   Solenoids per channel: 4
   PWM frequency: 25000 Hz
   Multiplex frequency: 200 Hz
   Enable pins: P1.4, P1.5
   Mode: Hardware Test

🚀 Starting initialization...

⚡ Initializing PWM controller...
  ✓ PWM device ready
  ✓ PWM frequency: 25000 Hz
  ✓ PWM period: 40 µs
  ✓ Number of channels: 4
  ✓ All PWM channels set to OFF
✓ PWM initialized

🔌 Setting up enable pins...
  ✓ Enable pin 0 (P1.4) configured
  ✓ Enable pin 1 (P1.5) configured
✓ Enable pins ready

📋 Initializing solenoid mappings...
  Solenoid  0: PWM Ch 0, Position 0, Note  60 (C)
  Solenoid  1: PWM Ch 0, Position 1, Note  61 (C#)
  ...
```

## Troubleshooting

### No Serial Output

If you see nothing:

1. **Check baud rate**: Must be **1000000** (1 Mbaud)
2. **Try pressing RESET** button on the board
3. **Check USB cable**: Ensure it's connected
4. **Try different COM port**: Board may have multiple ports, try each one
5. **Check firmware flashed**: Re-flash the firmware

### Garbage Characters

If you see weird symbols/gibberish:
- **Wrong baud rate!** Must be exactly **1000000**

### "Port is busy" or "Access denied"

- **Close other serial monitors** that might be using the port
- **Disconnect and reconnect** the USB cable
- **Restart VS Code**

## Next Steps After Opening Serial Monitor

Once you have serial monitor working:
1. **Press RESET** on the board to restart
2. **Watch for initialization messages**
3. **Look for solenoid/LED activation messages** like:
   ```
   🎹 Solenoid  0 (C) ON  - Power: 50% (velocity 64) [KICK+HOLD]
   ```
4. Report back what you see!

This will help us determine if:
- Firmware is running but LEDs not connected correctly
- Firmware is running but PWM pins not working
- Firmware isn't starting at all
- Some other issue
