/*
// HDC2080.h
// Description: This header file accompanies HDC2080.c
*/
#include <stdint.h>

/* Common addresses definition for temperature sensor. */
#define HDC2080_ADDRESS 0x40

// Define Register Map
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

// Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT 1
#define NINE_BIT 2

// Constants for setting sensor mode
#define TEMP_AND_HUMID 0
#define TEMP_ONLY 1
#define HUMID_ONLY 2
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1
#define LEVEL_MODE 0
#define COMPARATOR_MODE 1

// Constants for setting sample rate
#define MANUAL 0
#define TWO_MINS 1
#define ONE_MINS 2
#define TEN_SECONDS 3
#define FIVE_SECONDS 4
#define ONE_HZ 5
#define TWO_HZ 6
#define FIVE_HZ 7

float read_temp(void);                 // Returns the temperature in degrees C
uint8_t read_temp_offset_adjust(void); // Returns the offset adjust in binary
uint8_t set_temp_offset_adjust(
    uint8_t value);        // Set and return the offset adjust in binary
float read_humidity(void); // Returns the relative humidity
uint8_t
read_humidity_offset_adjust(void); // Returns the offset adjust in binary
uint8_t set_humidity_offset_adjust(
    uint8_t value);             // Set and return the offset adjust in binary
void enable_heater(void);       // Enables the heating element
void disable_heater(void);      // Disables the heating element
void set_low_temp(float temp);  // Sets low threshold temperature (in c)
void set_high_temp(float temp); // Sets high threshold temperature (in c)
void set_high_humidity(float humid); // Sets high Humiditiy threshold
void set_low_humidity(float humid);  // Sets low Humidity threshold
float read_low_humidity_threshold(
    void); // Returns contents of low humidity threshold register
float read_high_humidity_threshold(
    void); // Returns contents of high humidity threshold register
float read_low_temp_threshold(
    void); // Returns contents of low temperature threshold register (in C)
float read_high_temp_threshold(
    void); // Returns contents of high temperature threshold register (in C)
void trigger_measurement(
    void);                    // Triggers a manual temperature/humidity reading
void reset(void);             // Triggers a software reset
void enable_interrupt(void);  // Enables the interrupt/DRDY pin
void disable_interrupt(void); // Disables the interrupt/DRDY pin (High Z)
uint8_t
read_interrupt_status(void); // Reads the status of the interrupt register
void disable_threshold_interrupt(
    void); // Disables high and low temperature/humidity interrupts
void enable_drdy_interrupt(void);  // Enables data ready interrupt
void disable_drdy_interrupt(void); // Disables data ready interrupt
uint16_t read_manufacturer_id(
    void); // Reads a factory-programmable identification value that identifies
           // this device as being manufactured by Texas Instruments.
uint16_t read_device_id(void); // Reads a factory-programmable identification
                               // value that identifies this device as a HDC2080

void hdc2080_init(uint8_t hw_address);
void set_measurement_mode(int mode);
void set_humid_res(int resolution);
void set_temp_res(int resolution);