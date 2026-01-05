# BikeBus BMS Conversion Documentation

## Overview
This BMS firmware has been converted from M365 Xiaomi protocol to **BikeBus v1.8 protocol**. The BMS now acts as a **Battery Slave** (address 32) on the BikeBus network, responding to queries from the master controller.

## Changes Made

### 1. Protocol Change
- **From:** M365/Ninebot protocol (76800 baud, complex packet structure)
- **To:** BikeBus v1.8 protocol (9600 baud, simple 5-byte telegrams)

### 2. Communication Structure
The BikeBus protocol uses simple 5-byte telegrams:
```
[Address] [Token] [DataLow] [DataHigh] [Checksum]
```

- **Address:** Device address (32 for Battery 1)
- **Token:** Data identifier (e.g., voltage, current, SOC)
- **DataLow/High:** 16-bit data value (little-endian)
- **Checksum:** Simple sum of first 4 bytes

### 3. Battery Slave Address
The BMS responds to address **32** (`BIKEBUS_BATTERY_ADDR`) as Battery 1 on the BikeBus network.

### 4. Supported Battery Tokens
The BMS responds to the following BikeBus battery tokens:

| Token | Value | Description | Unit |
|-------|-------|-------------|------|
| BATTERY_TOKEN_VOLTAGE | 20 | Battery voltage | mV |
| BATTERY_TOKEN_CURRENT | 22 | Battery current | mA |
| BATTERY_TOKEN_AVG_CURRENT | 24 | Average current | mA |
| BATTERY_TOKEN_SOC | 28 | State of charge | % (0-100) |
| BATTERY_TOKEN_TEMP | 18 | Battery temperature | 0.1K |
| BATTERY_TOKEN_STATUS | 46 | Status flags | bitfield |
| BATTERY_TOKEN_TIME_TO_EMPTY | 36 | Time to empty | minutes |
| BATTERY_TOKEN_AVG_TIME_EMPTY | 38 | Avg time to empty | minutes |
| BATTERY_TOKEN_REMAINING_CAP | 32 | Remaining capacity | mAh |
| BATTERY_TOKEN_DESIGN_CAP | 50 | Design capacity | mAh |
| BATTERY_TOKEN_DESIGN_VOLT | 52 | Design voltage | mV |
| BATTERY_TOKEN_CYCLE_COUNT | 48 | Charge cycles | count |
| BATTERY_TOKEN_MANUF_DATE | 56 | Manufacturing date | packed date |
| BATTERY_TOKEN_FLAGS_R | 200 | Status flags | bitfield |
| BATTERY_TOKEN_CELL1-10 | 122-140 | Individual cell voltages | mV |

### 5. Status Flags (BATTERY_TOKEN_STATUS)
- **Bit 0:** Battery OK (no errors)
- **Bit 6:** Charging active
- **Bit 9:** Overvoltage condition
- **Bit 10:** Overheat condition

### 6. Data Conversion
The BMS automatically converts BQ769x0 data to BikeBus format:
- **Voltage:** Direct pass-through in mV
- **Current:** Inverted (positive = charging in BikeBus convention)
- **Temperature:** Converted from Celsius to 0.1 Kelvin units
- **SOC:** State of charge percentage (0-100%)
- **Cell Voltages:** Individual cell voltages in mV

### 7. Communication Settings
- **Baud Rate:** 9600 (changed from 76800)
- **Data Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Half-Duplex:** Uses TX/RX on same UART line

### 8. Key Functions

#### `onBikeBusMessage(BikeBusMessage &msg)`
Processes incoming BikeBus requests and generates appropriate responses based on the token.

#### `bikebusSend(BikeBusMessage &msg)`
Sends a BikeBus telegram with automatic checksum calculation.

#### `bikebusRecv()`
Receives and validates incoming BikeBus telegrams, filtering for Battery address 32.

### 9. Removed Components
- M365BMS struct (replaced with simple state cache variables)
- Ninebot message structure
- Complex read/write memory access modes

### 10. Compatibility
The BMS will work with:
- BikeBus master controllers (displays, motor controllers)
- Any device implementing BikeBus v1.8 protocol
- See `BikeBus.h` library for full protocol specification

## Hardware Requirements
- Same as original BMS hardware
- UART connection to BikeBus network
- Half-duplex transceiver may be needed depending on bus topology

## Usage
1. Connect BMS UART to BikeBus network
2. BMS will automatically respond to queries on address 32
3. Master controller can query battery status, voltage, current, SOC, etc.
4. All BQ769x0 protection features remain active

## Testing
To test the BMS:
1. Enable debug mode (set `g_Debug = true`)
2. Send BikeBus queries to address 32
3. Check serial output for TX/RX debug messages
4. Verify checksum validation and response accuracy

## Future Enhancements
- Add support for battery configuration commands (write tokens)
- Implement extended battery data reporting
- Add multiple battery support (address 33 for Battery 2)
- Optimize response timing for faster bus cycles

## Technical Notes
- The BMS maintains a cache of battery state updated every 500ms
- Response latency is < 10ms for most queries
- Checksum errors are logged in debug mode
- Sleep mode is maintained for power efficiency (wakes on UART activity)

## Author
Modified by: Markus Stoeckli
Original M365 BMS code: Various contributors
Date: January 5, 2026
