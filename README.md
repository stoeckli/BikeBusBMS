# BikeBus v1.8 Battery Management System (BMS)

#### Warning: This project is meant for people with electronics and microcontroller knowledge!

## Origin
This project is a **fork and conversion** of the excellent M365 BMS firmware originally created for Xiaomi M365 scooters.  
**Original Repository:** Various contributors to M365 BMS firmware  
**Converted by:** Markus Stoeckli (support@stoeckli.net)  
**Conversion Date:** January 5, 2026

## What Changed?
This firmware has been **completely converted** from the M365/Ninebot protocol to **BikeBus v1.8 protocol**.  
The BMS now acts as a **Battery Slave** (address 32) on the BikeBus network, making it compatible with BikeBus-enabled e-bike systems instead of Xiaomi scooters.

### Key Differences:
- **Protocol:** BikeBus v1.8 (5-byte telegrams) instead of M365/Ninebot
- **Baud Rate:** 9600 baud instead of 76800 baud
- **Communication:** Simple request/response via battery tokens
- **Compatibility:** Works with BikeBus master controllers (displays, motor controllers)
- **No M365 ESC patches needed** - this is a completely different protocol

## Overview
This repository contains BikeBus v1.8 compatible firmware for the following BMS hardware: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60-a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/).  
It's programmed with the [Arduino](https://www.arduino.cc/) platform and is built with [PlatformIO](https://platformio.org/).  
It runs on an ATMega328p MCU and controls a TI BQ769x0 ([TI Datasheet](http://www.ti.com/lit/ds/symlink/bq76940.pdf)) battery monitoring IC over I²C.

This BMS implements the **BikeBus v1.8 protocol** and acts as a Battery Slave, responding to queries from BikeBus master controllers with:

 * Battery SOC (State Of Charge: mAh, %) using Coulomb Counting
 * Pack Voltage, Cell Voltages, Current, Temperature
 * Discharge and Charge cycles
 * Individual cell voltages (Cell 1-10)
 * Battery status flags (charging, overvoltage, overheat)
 * Design capacity, nominal voltage, cycle count
 * Time to empty estimation

The battery information can be queried by any BikeBus v1.8 compatible master controller or display. See the [BikeBus library](../BikeBusDisplay/lib/BikeBus/) for the complete protocol specification.

## BikeBus Protocol Implementation

### Communication Settings
- **Baud Rate:** 9600 baud (8N1)
- **Protocol:** BikeBus v1.8 (5-byte telegrams)
- **BMS Address:** 32 (BIKEBUS_BATTERY_ADDR - Battery 1)
- **Half-Duplex:** Yes (single wire with transceiver)

### Telegram Format
```
[Address] [Token] [DataLow] [DataHigh] [Checksum]
```
- **Address:** Device address (32 for Battery 1)
- **Token:** Data identifier (voltage, current, SOC, etc.)
- **DataLow/High:** 16-bit data value (little-endian)
- **Checksum:** Simple sum of first 4 bytes

### Supported Battery Tokens
The BMS responds to the following BikeBus battery tokens:

| Token | Description | Unit | Notes |
|-------|-------------|------|-------|
| 18 | Battery temperature | 0.1K | (Kelvin * 10) |
| 20 | Battery voltage | mV | Total pack voltage |
| 22 | Battery current | mA | Positive = charging |
| 24 | Average current | mA | Smoothed value |
| 28 | State of charge | % | 0-100% |
| 32 | Remaining capacity | mAh | Based on SOC |
| 36 | Time to empty | minutes | Estimated |
| 38 | Avg time to empty | minutes | Estimated |
| 46 | Status flags | bitfield | See below |
| 48 | Cycle count | count | Charge cycles |
| 50 | Design capacity | mAh | Battery capacity |
| 52 | Design voltage | mV | Nominal voltage |
| 56 | Manufacturing date | packed | Date format |
| 122-140 | Cell voltages 1-10 | mV | Individual cells |
| 200 | Status flags read | bitfield | Same as token 46 |

### Status Flags (Token 46/200)
- **Bit 0:** Battery OK (no errors)
- **Bit 6:** Charging active
- **Bit 9:** Overvoltage condition
- **Bit 10:** Overheat condition

For complete protocol documentation, see [BIKEBUS_CONVERSION.md](BIKEBUS_CONVERSION.md).

## Hardware
### Requirements
* The BMS itself: [SP15SV0001-LLT](https://www.lithiumbatterypcb.com/product/13s-48v-li-ion-battery-pcb-board-54-6v-lithium-bms-with-60a-discharge-current-for-electric-motorcycle-and-e-scooter-protection-2-2-3-2-2-2-2-2/) or on [Aliexpress](https://www.aliexpress.com/item/12S-44-4V-smart-Lithium-li-ion-battery-protection-board-BMS-system-60A-20A-Bluetooth-phone/32976215661.html)
  * 10S - 12S. 30A version is recommended, >30A versions have both sides of the BMS PCB populated with MOSFETs and will not fit in the limited space of the M365.
* An ISP programmer with a 6pin ISP cable/adapter, example: [Aliexpress](https://www.aliexpress.com/item/10-Pin-Convert-to-Standard-6-Pin-Adapter-Board-USBASP-USBISP-AVR-Programmer-USB/2055099231.html)
* Serial UART adapter, sold by the BMS shop or on [Aliexpress](https://www.aliexpress.com/item/1PCS-CP2102-USB-2-0-to-TTL-UART-Module-6Pin-Serial-Converter-STC-Replace-FT232/32717057832.html)


### Current Shunt resistors
By default the BMS comes with ten 4mOhm shunt resistors in parallel. This results in a shunt resistance of 0.4mOhm, this is too small for accurate coulomb counting. In addition the voltage coming from the shunt resistors is also cut in half.

A simple fix for this is to simply remove some of the 4mOhm resistors so that only four resistors are left. Thus the shunt resistance will be 1mOhm.  
And to fix the shunt voltage from getting cut in half two more small resistors are removed from the top layer, as in the pictures: [Bottom](https://cloud.botox.bz/s/J6oZWqJDikzpTw8/preview) and [Top](https://cloud.botox.bz/s/2ipzTsJNWQ222TH/preview).

#### Current Shunt Resistors Calibration
One way to calibrate the BMS to know the exact current:

While measuring the charging current with a calibrated multimeter you can query the RAW current value via `configtool.py` using `debug_print()` command.
Now we make some calculations: ```R[uOhm]  = RAW Value * 8440 / A[mA]```.

 An example:
  * measured 4,1169A charging current
  * raw value of 445
  * Results in ```R =  445 * 8440 / 4116,9 = 911,69```
  * used in Settings: ```g_Settings.shuntResistor_uOhm = 912```. 

_Please refer to [Software/Configuration](#Configuration) on how to get, change, put, apply and save Settings for the BMS._

### Wiring
#### IMPORTANT: Connect the BMS to your BikeBus network properly!

**The BikeBus uses half-duplex UART communication. You may need a half-duplex transceiver depending on your setup.**

**Connection Points:**
- **TX/RX:** Connect to the BikeBus data line (half-duplex)
- **P-:** Connect to battery negative (main discharge path)
- **C-:** Use for charging (charge protection)
- **B-:** Connect to battery pack negative

**BikeBus Communication:**
- The BMS listens on address 32 (Battery 1)
- Master controller queries the BMS for battery data
- BMS responds with voltage, current, SOC, temperature, etc.
- All communication is at 9600 baud

**Important Notes:**
- Do not connect GND separately if using half-duplex transceivers with isolation
- Ensure proper ground reference between BMS and BikeBus master
- P- carries the main discharge current
- C- should be used for charging to enable overvoltage protection

## Software
### Configuration
Depending on your battery you might want to configure some of the settings here: [src/main.h](src/main.h#L24)

* `capacity`: The *actual* total capacity in mAh of your battery pack, don't use the manufacturer stated capacity if you want to have accurate SOC but rather look at sites like [lygte-info.dk](https://lygte-info.dk/), etc.
  * Example: for 4 * NCR18650B I used a value of 12400mAh.
* `nominal_voltage`: The nominal voltage in mV of your cells, this will be 3.6V for almost all cells.
* `full_voltage`: The voltage in mV that you will charge your cells to. I only charge mine to 4.1V for cycle life.
* `ODP_current`: Over current protection value in mA, if your system is shutting off then make this higher.
* `UVP_voltage`: The BMS will shut off P- when any cells voltage goes below this.
  * Remember: During load the cell voltage will drop a lot.
* `OVP_voltage`: The BMS will shut off C- when any cells voltage goes above this.
  * Use C- for charging to enable overvoltage protection!

**Note:** This BikeBus BMS does not use the `configtool.py` from the M365 version. Configuration must be done by editing `src/main.h` and recompiling.

### BikeBus Protocol Details
For detailed information about the BikeBus v1.8 protocol implementation, see:
- [BIKEBUS_CONVERSION.md](BIKEBUS_CONVERSION.md) - Complete conversion documentation
- [BikeBus Library](../BikeBusDisplay/lib/BikeBus/) - Protocol specification and examples

### Compiling
This project uses [PlatformIO](https://platformio.org/), please check out their [Documentation](https://docs.platformio.org/en/latest/) to get started.

### Programming bootloader via ISP
Connect your ISP programmer to the BMS, you can (and should) keep the battery disconnected while programming.  
Here's the pinout of the ISP header: [Image](https://cloud.botox.bz/s/qGa7rS6Ktt4pG24/preview)  
On the new V1.5 PCB it's a little trickier: [Image1](https://cloud.botox.bz/s/eYmBCM4Z44P84tj/preview) [Image2](https://cloud.botox.bz/s/7BkSS7NKk878B4d/preview)

Choose the correct bootloader file (the MHZ value in the Filename **must** match your hardware, default = 8MHz)

Choose the correct fuses for your hardware
* for internal 8MHz Clock use ```-U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m``` (Hardware version <= 1.5)
* for external 8MHz Clock use ```-U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m``` (Hardware version >= 1.6)
* for anything else think twice what values you are using as you can brick your atmega chip with wrong values, use of tools like [AVR Fuse Calculator](http://www.engbedded.com/fusecalc) is recommended.

Finally flash the bootloader with [AVRDUDE](https://
download.savannah.gnu.org/releases/avrdude/avrdude-6.3-mingw32.zip) using the following command: `avrdude -patmega328p -cstk500v2 -P/dev/ttyUSB0 -U lfuse:w:0xE2:m -U hfuse:w:0xDE:m -U efuse:w:0xFD:m -U lock:w:0x3F:m -U flash:w:optiboot_atmega328_8mhz_57600bps.hex`  
* Adjust the `-P/dev/ttyUSB0` part to the correct COM port on your PC. (Omit for `usbasp`)
* Adjust the `-cstk500v2` part to your programmer (`-cusbasp` for the [Aliexpress](https://www.aliexpress.com/item/10-Pin-Convert-to-Standard-6-Pin-Adapter-Board-USBASP-USBISP-AVR-Programmer-USB/2055099231.html) example one.)
* Adjust the `-U` parts with your fuse values

### Uploading/Updating firmware
You can upload the firmware in platformio, there's a little arrow somewhere.  
You have to short the RESET pin to GROUND right before you hit the upload button!  
For updating firmware you don't have to short the RESET pin but can run `bootloader.py /dev/ttyUSB0` right before you hit upload.

**Note:** The BikeBus firmware uses **9600 baud**, not 76800 or 57600 like the M365 version.

### Optional: Building optiboot yourself
Clone the [Optiboot repository](https://github.com/Optiboot/optiboot/) and `git apply` this patch: [optiboot.diff](optiboot.diff)
* 8MHz build: `make atmega328 AVR_FREQ=8000000L PRODUCTION=1 BAUD_RATE=57600 LED_START_FLASHES=0`
* 16MHz build: `make atmega328 AVR_FREQ=16000000L PRODUCTION=1 BAUD_RATE=115200 LED_START_FLASHES=0`


## Troubleshooting

**This BMS uses BikeBus protocol, not M365 protocol!** Do not try to connect it to a Xiaomi M365 scooter.

Make sure the temperature sensors are plugged in and all wires are connected properly.
B- needs to be connected to the battery -.

Reset the BMS by shorting GND with RST on the ISP header.

Does your Voltmeter only show a too low voltage like 10 or 20 Volt? -> Reset the BMS by shorting GND with RST on the ISP header. The AVR Chip crashed during plugging the balancing connector.

### Testing BikeBus Communication
You can test the BMS by sending BikeBus telegrams:
1. Connect a USB-UART adapter to the BMS TX/RX pins
2. Set baud rate to 9600, 8N1
3. Send a query: `[32][20][00][00][52]` (hex) - Query battery voltage
4. BMS should respond with: `[32][20][LL][HH][CS]` where LL/HH is voltage in mV

Enable debug mode in the code (`g_Debug = true`) to see detailed TX/RX messages on the serial console.

### Status Messages
On boot, the BMS prints: `BikeBus BMS BOOTED!`

## BikeBus Integration
The BMS is designed to work with BikeBus v1.8 compatible master controllers. For example applications, see:
- [BikeBusDisplay](../BikeBusDisplay/) - Display controller with BikeBus library
- The master controller should query the BMS periodically for battery data
- Typical query sequence: voltage → current → SOC → temperature → status

### Connecting to a BikeBus Master
1. Connect BMS UART to the BikeBus network (may require half-duplex transceiver)
2. Ensure proper ground reference
3. Master controller should query address 32 (Battery 1)
4. BMS will automatically respond to valid queries
5. Data is updated every 500ms internally

## Compatibility
- **Works with:** BikeBus v1.8 compatible controllers, displays, and systems
- **Does NOT work with:** Xiaomi M365 scooters (different protocol)
- **Battery Protection:** All BQ769x0 features remain active (OV, UV, OC, temperature)
- **Tested with:** BikeBus library (see BikeBusDisplay project)


## Final words
### Credits
**Original M365 BMS firmware:** Various contributors to the M365 open-source community  
**BQ769x0 library:** A big part of the BQ769x0 code is taken from [LibreSolar/bq769x0_mbed_lib](https://github.com/LibreSolar/bq769x0_mbed_lib)  
**BikeBus v1.8 Protocol Conversion:** Markus Stoeckli (support@stoeckli.net) - January 2026

This firmware is a complete protocol conversion from M365/Ninebot to BikeBus v1.8, making the BMS compatible with BikeBus e-bike systems.

### Support
For BikeBus-specific questions: contact Markus Stoeckli (support@stoeckli.net)  
For BQ769x0 hardware questions: see original M365 BMS community resources

### Related Projects
- [BikeBusDisplay](../BikeBusDisplay/) - Display controller with BikeBus v1.8 library
- [BikeBus Protocol Specification](../BikeBusDisplay/lib/BikeBus/) - Complete protocol documentation

### Disclaimer
If you break anything it's your own fault.  
**Works for me™.** is the only guarantee I can give you.

I am in no way affiliated with the company that makes the BMS. I just bought it, reversed some stuff and made this firmware.  
This is a conversion of existing open-source M365 BMS firmware to the BikeBus v1.8 protocol.

### License
This project maintains the same license as the original M365 BMS firmware from which it was derived.  
BikeBus protocol implementation: Copyright (C) 2026 Markus Stoeckli
 
