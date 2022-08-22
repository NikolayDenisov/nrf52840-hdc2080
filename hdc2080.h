/*
// HDC2080.h
// Description: This header file accompanies HDC2080.c
*/
#include <stdint.h>

/* Common addresses definition for temperature sensor. */
#define HDC2080_ADDRESS          0x40


//Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define INT_CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

//  Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT 1
#define NINE_BIT 2

//  Constants for setting sensor mode
#define TEMP_AND_HUMID 0
#define TEMP_ONLY 1
#define HUMID_ONLY 2
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1
#define LEVEL_MODE 0
#define COMPARATOR_MODE 1

//  Constants for setting sample rate
#define MANUAL 0
#define TWO_MINS 1
#define ONE_MINS 2
#define TEN_SECONDS 3
#define FIVE_SECONDS 4
#define ONE_HZ 5
#define TWO_HZ 6
#define FIVE_HZ 7

float readTemp(void);	// Returns the temperature in degrees C
uint8_t readTempOffsetAdjust(void);	// Returns the offset adjust in binary
uint8_t setTempOffsetAdjust(uint8_t value);	// Set and return the offset adjust in binary
float readHumidity(void);	// Returns the relative humidity
uint8_t readHumidityOffsetAdjust(void);	// Returns the offset adjust in binary
uint8_t setHumidityOffsetAdjust(uint8_t value);	// Set and return the offset adjust in binary
void enableHeater(void);	// Enables the heating element
void disableHeater(void);	// Disables the heating element
void setLowTemp(float temp);	// Sets low threshold temperature (in c)
void setHighTemp(float temp);	// Sets high threshold temperature (in c)
void setHighHumidity(float humid);	// Sets high Humiditiy threshold
void setLowHumidity(float humid);	// Sets low Humidity threshold
float readLowHumidityThreshold(void);	// Returns contents of low humidity threshold register
float readHighHumidityThreshold(void);	// Returns contents of high humidity threshold register
float readLowTempThreshold(void);	// Returns contents of low temperature threshold register (in C)
float readHighTempThreshold(void);	// Returns contents of high temperature threshold register (in C)
void triggerMeasurement(void);	// Triggers a manual temperature/humidity reading
void reset(void);	// Triggers a software reset
void enableInterrupt(void);	// Enables the interrupt/DRDY pin
void disableInterrupt(void);	// Disables the interrupt/DRDY pin (High Z)
uint8_t readInterruptStatus(void);	// Reads the status of the interrupt register
void disableThresholdInterrupt(void);	// Disables high and low temperature/humidity interrupts
void enableDRDYInterrupt(void);	// Enables data ready interrupt
void disableDRDYInterrupt(void);	// Disables data ready interrupt
uint16_t readManufacturerId(void);      // Reads a factory-programmable identification value that identifies this device as being manufactured by Texas Instruments.
uint16_t readDeviceId(void);            // Reads a factory-programmable identification value that identifies this device as a HDC2080