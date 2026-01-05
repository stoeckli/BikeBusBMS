#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "bq769x0.h"

/* Software I2C */
#define I2C_TIMEOUT 100
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7
/* Software I2C */

#define BMS_ALERT_PIN 17 // 17 = ALERT, PC3, PCINT11
#define BMS_BOOT_PIN 14 // 14 = BOOT, PC0
#define BMS_I2C_FET_PIN 8 // 8 = I2C Pull-Up FET Gate, PB0
#define BMS_K1_PIN 13 // 13 = PB5 = K1 connector
#define BMS_VDD_EN_PIN 15 // 15 = PC1 = VDD Enable

// BikeBus Protocol Defines
#define BIKEBUS_BATTERY_ADDR 32  // Battery 1 address on BikeBus

// BikeBus Protocol Defines
#define BIKEBUS_BATTERY_ADDR 32  // Battery 1 address on BikeBus

// Battery Tokens (from BikeBus protocol)
#define BATTERY_TOKEN_MODE          8
#define BATTERY_TOKEN_TEMP          18
#define BATTERY_TOKEN_VOLTAGE       20
#define BATTERY_TOKEN_CURRENT       22
#define BATTERY_TOKEN_AVG_CURRENT   24
#define BATTERY_TOKEN_MAX_ERROR     26
#define BATTERY_TOKEN_SOC           28
#define BATTERY_TOKEN_REMAINING_CAP 32
#define BATTERY_TOKEN_TIME_TO_EMPTY 36
#define BATTERY_TOKEN_AVG_TIME_EMPTY 38
#define BATTERY_TOKEN_AVG_TIME_FULL 40
#define BATTERY_TOKEN_CURRENT_LIMIT_R 44
#define BATTERY_TOKEN_STATUS        46
#define BATTERY_TOKEN_CYCLE_COUNT   48
#define BATTERY_TOKEN_DESIGN_CAP    50
#define BATTERY_TOKEN_DESIGN_VOLT   52
#define BATTERY_TOKEN_MANUF_DATE    56
#define BATTERY_TOKEN_SERIAL        58
#define BATTERY_TOKEN_CELL1         122
#define BATTERY_TOKEN_CELL2         124
#define BATTERY_TOKEN_CELL3         126
#define BATTERY_TOKEN_CELL4         128
#define BATTERY_TOKEN_CELL5         130
#define BATTERY_TOKEN_CELL6         132
#define BATTERY_TOKEN_CELL7         134
#define BATTERY_TOKEN_CELL8         136
#define BATTERY_TOKEN_CELL9         138
#define BATTERY_TOKEN_CELL10        140
#define BATTERY_TOKEN_EXTENDED_DATA 164
#define BATTERY_TOKEN_FLAGS_R       200
#define BATTERY_TOKEN_FLAGS_W       201

struct BMSSettings
{
    uint8_t header[2] = {0xB0, 0x0B};
    uint16_t version = 1;
    char serial[14] = "BOTOX001";
    uint32_t capacity = 7800; // mAh
    uint16_t nominal_voltage = 3600; // mV
    uint16_t full_voltage = 4150; // mV
    uint16_t num_cycles = 0;
    uint16_t num_charged = 0;
    uint16_t date = (18 << 9) | (10 << 5) | 1; // MSB (7 bits year, 4 bits month, 5 bits day) LSB

    // setShuntResistorValue
    uint16_t shuntResistor_uOhm = 1000;

    // setThermistorBetaValue
    uint16_t thermistor_BetaK = 3435;

    // setTemperatureLimits
    int16_t temp_minDischargeC = -20; // °C
    int16_t temp_maxDischargeC = 60; // °C
    int16_t temp_minChargeC = 0; // °C
    int16_t temp_maxChargeC = 45; // °C

    // setShortCircuitProtection
    uint32_t SCD_current = 80000; // mA
    uint16_t SCD_delay = 200; // us

    // setOvercurrentChargeProtection
    uint32_t OCD_current = 6000; // mA
    uint16_t OCD_delay = 3000; // ms

    // setOvercurrentDischargeProtection
    uint32_t ODP_current = 35000; // mA
    uint16_t ODP_delay = 1280; // ms

    // setCellUndervoltageProtection
    uint16_t UVP_voltage = 2800; // mV
    uint16_t UVP_delay = 2; // s

    // setCellOvervoltageProtection
    uint16_t OVP_voltage = 4200; // mV
    uint16_t OVP_delay = 2; // s

    // setBalancingThresholds
    uint16_t balance_minIdleTime = 1800; // s
    uint16_t balance_minVoltage = 3600; // mV
    uint16_t balance_maxVoltageDiff = 10; // mV

    // setIdleCurrentThreshold
    uint16_t idle_currentThres = 500; // mA

    // enableAutoBalancing
    uint16_t balance_enabled = 1;

    // adjADCPackOffset
    int16_t adcPackOffset = 0;

    // adjADCCellsOffset
    int16_t adcCellsOffset[15] = {0};

} __attribute__((packed));

/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/
/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/
/*** DON'T CHANGE ANYTHING BELOW THIS LINE ***/

// BikeBus Message Structure
struct BikeBusMessage
{
    uint8_t address;     // Device address
    uint8_t token;       // Data token/command
    uint8_t dataLow;     // Low byte of data
    uint8_t dataHigh;    // High byte of data
    uint8_t checksum;    // Simple sum checksum
};

void alertISR();

void loadSettings();
void saveSettings();
void applySettings();

void onBikeBusMessage(BikeBusMessage &msg);
void bikebusSend(BikeBusMessage &msg);
void bikebusRecv();

void debug_print();

extern bool g_Debug;
extern bq769x0 g_BMS;
extern volatile bool g_interruptFlag;
extern unsigned long g_lastActivity;
extern unsigned long g_lastUpdate;

#endif // MAIN_H
