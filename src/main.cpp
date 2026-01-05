#include <Arduino.h>
#include <limits.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include "bq769x0.h"
#include "main.h"

BMSSettings g_Settings;

bool g_Debug = true;
// I2CAddress = 0x08, crcEnabled = true
bq769x0 g_BMS(bq76940, 0x08, true);
volatile bool g_interruptFlag = false;
unsigned long g_lastActivity = 0;
unsigned long g_lastUpdate = 0;
volatile bool g_uartRxInterrupted = false;
volatile bool g_wakeupFlag = false;

unsigned long g_oldMillis = 0;
int g_millisOverflows = 0;

extern volatile unsigned long timer0_millis;
volatile unsigned int g_timer2Overflows = 0;

// BikeBus battery state cache
uint16_t g_batteryVoltage = 0;      // in mV
int16_t g_batteryCurrent = 0;       // in mA
int16_t g_batteryAvgCurrent = 0;    // in mA
uint8_t g_batterySOC = 0;           // State of Charge in %
uint16_t g_batteryTemp = 0;         // in 0.1K
uint16_t g_batteryStatus = 0;       // Status flags
uint16_t g_batteryTimeToEmpty = 0;  // in minutes
uint16_t g_cellVoltages[15] = {0};  // Cell voltages in mV

void alertISR()
{
    g_BMS.setAlertInterruptFlag();
    g_interruptFlag = true;
    g_wakeupFlag = true;
}

void uartRxISR()
{
    g_uartRxInterrupted = true;
    g_wakeupFlag = true;
}

ISR(TIMER2_OVF_vect)
{
    // only used to keep track of time while sleeping to adjust millis()
    g_timer2Overflows++;
}

void setup()
{
    MCUSR = 0;
    wdt_disable();

    Serial.begin(9600); // BikeBus uses 9600 baud
    Serial.println(F("BikeBus BMS BOOTED!"));

    power_adc_disable();
    power_spi_disable();
    power_timer1_disable();
    power_twi_disable();
    delay(1000);

    loadSettings();

    g_BMS.begin(BMS_BOOT_PIN);
    g_BMS.setThermistors(0b110);

    applySettings();

    // Bluetooth power pin
    pinMode(BMS_VDD_EN_PIN, OUTPUT);

    // attach ALERT interrupt
    pinMode(BMS_ALERT_PIN, INPUT);
    attachPCINT(digitalPinToPCINT(BMS_ALERT_PIN), alertISR, RISING);

    // attach UART RX pin interrupt to wake from deep sleep
    attachPCINT(digitalPinToPCINT(0), uartRxISR, CHANGE);
    disablePCINT(digitalPinToPCINT(0));

    interrupts();

    delay(1000);
    g_BMS.update();
    g_BMS.resetSOC(100);

    g_BMS.enableDischarging();
    g_BMS.enableCharging();

    g_Debug = false;

    wdt_enable(WDTO_1S);
}


void loadSettings()
{
    if(EEPROM.read(0) != g_Settings.header[0] || EEPROM.read(1) != g_Settings.header[1])
    {
        for(uint16_t i = 0; i < EEPROM.length(); i++) {
            EEPROM.write(i, 0);
        }
        EEPROM.put(0, g_Settings);
    }
    else
    {
        EEPROM.get(0, g_Settings);
    }
}

void saveSettings()
{
    EEPROM.put(0, g_Settings);
}

void applySettings()
{
    g_BMS.setBatteryCapacity(g_Settings.capacity, g_Settings.nominal_voltage, g_Settings.full_voltage);

    g_BMS.setShuntResistorValue(g_Settings.shuntResistor_uOhm);
    g_BMS.setThermistorBetaValue(g_Settings.thermistor_BetaK);

    g_BMS.setTemperatureLimits(g_Settings.temp_minDischargeC,
                             g_Settings.temp_maxDischargeC,
                             g_Settings.temp_minChargeC,
                             g_Settings.temp_maxChargeC);
    g_BMS.setShortCircuitProtection(g_Settings.SCD_current, g_Settings.SCD_delay);
    g_BMS.setOvercurrentChargeProtection(g_Settings.OCD_current, g_Settings.OCD_delay);
    g_BMS.setOvercurrentDischargeProtection(g_Settings.ODP_current, g_Settings.ODP_delay);
    g_BMS.setCellUndervoltageProtection(g_Settings.UVP_voltage, g_Settings.UVP_delay);
    g_BMS.setCellOvervoltageProtection(g_Settings.OVP_voltage, g_Settings.OVP_delay);

    g_BMS.setBalancingThresholds(g_Settings.balance_minIdleTime,
                               g_Settings.balance_minVoltage,
                               g_Settings.balance_maxVoltageDiff);
    g_BMS.setIdleCurrentThreshold(g_Settings.idle_currentThres);
    if(g_Settings.balance_enabled)
        g_BMS.enableAutoBalancing();
    else
        g_BMS.disableAutoBalancing();

    g_BMS.setBalanceCharging(true);

    g_BMS.adjADCPackOffset(g_Settings.adcPackOffset);
    g_BMS.adjADCCellsOffset(g_Settings.adcCellsOffset);
}


void onBikeBusMessage(BikeBusMessage &msg)
{
    // Enable TX
    UCSR0B |= (1 << TXEN0);

    if(msg.address != BIKEBUS_BATTERY_ADDR)
        return;

    // Prepare response message
    BikeBusMessage response;
    response.address = BIKEBUS_BATTERY_ADDR;
    response.token = msg.token;
    
    uint16_t value = 0;

    // Handle different token requests
    switch(msg.token)
    {
        case BATTERY_TOKEN_VOLTAGE:
            value = g_batteryVoltage;
            break;
            
        case BATTERY_TOKEN_CURRENT:
            value = (uint16_t)g_batteryCurrent;
            break;
            
        case BATTERY_TOKEN_AVG_CURRENT:
            value = (uint16_t)g_batteryAvgCurrent;
            break;
            
        case BATTERY_TOKEN_SOC:
            value = g_batterySOC;
            break;
            
        case BATTERY_TOKEN_TEMP:
            value = g_batteryTemp;
            break;
            
        case BATTERY_TOKEN_STATUS:
            value = g_batteryStatus;
            break;
            
        case BATTERY_TOKEN_AVG_TIME_EMPTY:
        case BATTERY_TOKEN_TIME_TO_EMPTY:
            value = g_batteryTimeToEmpty;
            break;
            
        case BATTERY_TOKEN_REMAINING_CAP:
            value = (uint16_t)(g_Settings.capacity * g_batterySOC / 100);
            break;
            
        case BATTERY_TOKEN_DESIGN_CAP:
            value = g_Settings.capacity;
            break;
            
        case BATTERY_TOKEN_DESIGN_VOLT:
            value = g_Settings.nominal_voltage;
            break;
            
        case BATTERY_TOKEN_CYCLE_COUNT:
            value = g_Settings.num_cycles;
            break;
            
        case BATTERY_TOKEN_MANUF_DATE:
            value = g_Settings.date;
            break;
            
        case BATTERY_TOKEN_FLAGS_R:
            // Return status flags
            value = g_batteryStatus;
            break;
            
        // Cell voltages
        case BATTERY_TOKEN_CELL1:
            value = g_cellVoltages[0];
            break;
        case BATTERY_TOKEN_CELL2:
            value = g_cellVoltages[1];
            break;
        case BATTERY_TOKEN_CELL3:
            value = g_cellVoltages[2];
            break;
        case BATTERY_TOKEN_CELL4:
            value = g_cellVoltages[3];
            break;
        case BATTERY_TOKEN_CELL5:
            value = g_cellVoltages[4];
            break;
        case BATTERY_TOKEN_CELL6:
            value = g_cellVoltages[5];
            break;
        case BATTERY_TOKEN_CELL7:
            value = g_cellVoltages[6];
            break;
        case BATTERY_TOKEN_CELL8:
            value = g_cellVoltages[7];
            break;
        case BATTERY_TOKEN_CELL9:
            value = g_cellVoltages[8];
            break;
        case BATTERY_TOKEN_CELL10:
            value = g_cellVoltages[9];
            break;
            
        default:
            // Unknown token - send error response (token 0x00)
            response.token = 0x00;
            value = 0;
            break;
    }

    // Send response
    response.dataLow = value & 0xFF;
    response.dataHigh = (value >> 8) & 0xFF;
    bikebusSend(response);
}

void bikebusSend(BikeBusMessage &msg)
{
    // Calculate checksum (simple sum of first 4 bytes)
    msg.checksum = msg.address + msg.token + msg.dataLow + msg.dataHigh;
    
    Serial.write(msg.address);
    Serial.write(msg.token);
    Serial.write(msg.dataLow);
    Serial.write(msg.dataHigh);
    Serial.write(msg.checksum);
    Serial.flush(); // Wait for transmission to complete
    
    // Debug: Print sent message
    if (g_Debug) {
        Serial.print(F("TX: "));
        Serial.print(msg.address, HEX);
        Serial.print(F(" "));
        Serial.print(msg.token, HEX);
        Serial.print(F(" "));
        Serial.print(msg.dataLow, HEX);
        Serial.print(F(" "));
        Serial.print(msg.dataHigh, HEX);
        Serial.print(F(" "));
        Serial.println(msg.checksum, HEX);
    }
    
    // Clear receive buffer to remove echo of transmitted bytes
    delay(2);
    while (Serial.available()) {
        Serial.read();
    }
}

void bikebusRecv()
{
    static BikeBusMessage msg;
    static uint8_t recvd = 0;
    static unsigned long begin = 0;

    while(Serial.available())
    {
        g_lastActivity = millis();

        if(millis() >= begin + 50)
        { // 50ms timeout - reset if too much time has passed
            recvd = 0;
        }

        uint8_t byte = Serial.read();
        
        switch(recvd)
        {
            case 0: // Address byte
            {
                msg.address = byte;
                begin = millis();
                recvd++;
            } break;

            case 1: // Token byte
            {
                msg.token = byte;
                recvd++;
            } break;

            case 2: // Data low byte
            {
                msg.dataLow = byte;
                recvd++;
            } break;

            case 3: // Data high byte
            {
                msg.dataHigh = byte;
                recvd++;
            } break;

            case 4: // Checksum byte
            {
                msg.checksum = byte;
                
                // Verify checksum
                uint8_t expectedChecksum = msg.address + msg.token + msg.dataLow + msg.dataHigh;
                if(msg.checksum == expectedChecksum)
                {
                    // Process valid message
                    onBikeBusMessage(msg);
                }
                else if(g_Debug)
                {
                    Serial.print(F("Checksum error! Expected: "));
                    Serial.print(expectedChecksum, HEX);
                    Serial.print(F(", Got: "));
                    Serial.println(msg.checksum, HEX);
                }
                
                recvd = 0;
            } break;
        }
    }
}


void loop()
{
    unsigned long now = millis();
    if(g_interruptFlag || (unsigned long)(now - g_lastUpdate) >= 500)
    {
        if(g_interruptFlag)
            g_interruptFlag = false;

        uint8_t error = g_BMS.update(); // should be called at least every 250 ms
        g_lastUpdate = now;

        // Update BikeBus battery state cache
        {
            // Voltage in mV
            g_batteryVoltage = g_BMS.getBatteryVoltage();
            
            // Current in mA (positive = charging, negative = discharging)
            g_batteryCurrent = -g_BMS.getBatteryCurrent(); // Invert for BikeBus convention
            g_batteryAvgCurrent = g_batteryCurrent; // Could be averaged over time
            
            // State of charge (0-100%)
            g_batterySOC = g_BMS.getSOC();
            
            // Temperature in 0.1K (Kelvin * 10)
            // BMS gives us Celsius, convert to 0.1K: (C + 273.15) * 10
            float tempC = g_BMS.getTemperatureDegC(1);
            g_batteryTemp = (uint16_t)((tempC + 273.15) * 10.0);
            
            // Status flags
            g_batteryStatus = 0;
            if(g_BMS.getBatteryCurrent() > (int16_t)g_Settings.idle_currentThres)
                g_batteryStatus |= (1 << 6); // Charging
            if(error & STAT_OV)
                g_batteryStatus |= (1 << 9); // Overvoltage
            if(g_BMS.getHighestTemperature() > (g_Settings.temp_maxDischargeC - 3) * 10)
                g_batteryStatus |= (1 << 10); // Overheat
            if(!error)
                g_batteryStatus |= 1; // OK status
            
            // Estimate time to empty (rough calculation based on capacity and current)
            if(g_batteryCurrent < -100) // If discharging
            {
                uint16_t remainingCapacity = g_Settings.capacity * g_batterySOC / 100;
                g_batteryTimeToEmpty = (remainingCapacity * 60) / (-g_batteryCurrent); // minutes
            }
            else
            {
                g_batteryTimeToEmpty = 0xFFFF; // Not discharging
            }
            
            // Update cell voltages
            uint8_t numCells = g_BMS.getNumberOfConnectedCells();
            for(uint8_t i = 0; i < numCells && i < 15; i++)
                g_cellVoltages[i] = g_BMS.getCellVoltage(i);
            
            // Handle cycle counting
            if(g_BMS.batCycles_) {
                g_Settings.num_cycles += g_BMS.batCycles_;
                g_BMS.batCycles_ = 0;
                EEPROM.put(0, g_Settings);
            }

            if(g_BMS.chargedTimes_) {
                g_Settings.num_charged += g_BMS.chargedTimes_;
                g_BMS.chargedTimes_ = 0;
                EEPROM.put(0, g_Settings);
            }

            // Cell voltage difference check
            uint16_t bigDelta = g_BMS.getMaxCellVoltage() - g_BMS.getMinCellVoltage();
            if(bigDelta > 100)
                error = 1;
        }

        if(g_oldMillis > now)
            g_millisOverflows++;
        g_oldMillis = now;
    }

    bikebusRecv();

    if((unsigned long)(now - g_lastActivity) >= 5000 && !g_Debug)
    {
        // Disable TX
        UCSR0B &= ~(1 << TXEN0);

        // go into deep sleep, will wake up every 250ms by BQ769x0 ALERT or from USART1 RX (first byte will be lost)
        noInterrupts();
        set_sleep_mode(SLEEP_MODE_PWR_SAVE);

        // Timer/Counter2 8-byte OVF 8MHz /1024 = 32.64ms
        TCCR2A = 0;
        TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);
        TCNT2 = 0;
        TIMSK2 = (1<<TOIE2);

        UCSR0B &= ~(1 << RXEN0); // Disable RX
        enablePCINT(digitalPinToPCINT(0));

        wdt_reset();
        g_wakeupFlag = false;

        sleep_enable();
        interrupts();
        do // go to sleep if it's just timer2 that woke us up (unless we were idle for longer than 500ms)
        {
            sleep_cpu();
        } while(!g_wakeupFlag && g_timer2Overflows < 16);
        sleep_disable();

        // Disable Timer/Counter2 and add elapsed time to Arduinos 'timer0_millis'
        TCCR2B = 0;
        TIMSK2 = 0;
        float elapsed_time = g_timer2Overflows * 32.64 + TCNT2 * 32.64 / 255.0;
        timer0_millis += (unsigned long)elapsed_time;
        g_timer2Overflows = 0;

        if(g_uartRxInterrupted)
            g_lastActivity = millis();
        g_uartRxInterrupted = false;

        disablePCINT(digitalPinToPCINT(0));
        UCSR0B |= (1 << RXEN0); // Enable RX

        interrupts();
    }

    wdt_reset();
}

#if BQ769X0_DEBUG
void debug_print()
{
    g_BMS.printRegisters();
    Serial.println(F(""));

    unsigned long uptime = g_millisOverflows * (UINT_MAX / 1000UL);
    uptime += millis() / 1000;
    Serial.print(F("uptime: "));
    Serial.println(uptime);

    Serial.print(F("Battery voltage: "));
    Serial.print(g_BMS.getBatteryVoltage());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryVoltage(true));
    Serial.println(F(")"));

    Serial.print(F("Battery current: "));
    Serial.print(g_BMS.getBatteryCurrent());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryCurrent(true));
    Serial.println(F(")"));

    Serial.print(F("SOC: "));
    Serial.println(g_BMS.getSOC());

    Serial.print(F("Temperature: "));
    Serial.print(g_BMS.getTemperatureDegC(1));
    Serial.print(F(" "));
    Serial.println(g_BMS.getTemperatureDegC(2));

    Serial.print(F("Balancing status: "));
    Serial.println(g_BMS.getBalancingStatus());

    Serial.print(F("Cell voltages ("));
    int numCells = g_BMS.getNumberOfCells();
    Serial.print(numCells);
    Serial.println(F("):"));
    for(int i = 0; i < numCells; i++) {
        Serial.print(g_BMS.getCellVoltage_(i));
        Serial.print(F(" ("));
        Serial.print(g_BMS.getCellVoltage_(i, true));
        Serial.print(F(")"));
        if(i != numCells - 1)
            Serial.print(F(", "));
    }
    Serial.println(F(""));

    Serial.print(F("Cell V: Min: "));
    Serial.print(g_BMS.getMinCellVoltage());
    Serial.print(F(" | Avg: "));
    Serial.print(g_BMS.getAvgCellVoltage());
    Serial.print(F(" | Max: "));
    Serial.print(g_BMS.getMaxCellVoltage());
    Serial.print(F(" | Delta: "));
    Serial.println(g_BMS.getMaxCellVoltage() - g_BMS.getMinCellVoltage());

    Serial.print(F("BikeBus Battery Voltage: "));
    Serial.println(g_batteryVoltage);
    Serial.print(F("BikeBus Battery Current: "));
    Serial.println(g_batteryCurrent);
    Serial.print(F("BikeBus Battery SOC: "));
    Serial.println(g_batterySOC);
    Serial.print(F("BikeBus Battery Temp: "));
    Serial.println(g_batteryTemp);
    Serial.print(F("BikeBus Battery Status: "));
    Serial.println(g_batteryStatus);

    Serial.print(F("XREADY errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_XREADY]);
    Serial.print(F("ALERT errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_ALERT]);
    Serial.print(F("UVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_UVP]);
    Serial.print(F("OVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OVP]);
    Serial.print(F("SCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_SCD]);
    Serial.print(F("OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OCD]);
    Serial.println();
    Serial.print(F("DISCHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_DISCHG_TEMP]);
    Serial.print(F("CHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_TEMP]);
    Serial.print(F("CHG OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_OCD]);
}
#endif
