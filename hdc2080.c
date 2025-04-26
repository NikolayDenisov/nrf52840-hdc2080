/*
// Description: This library facilitates communication with, and configuration
of, the HDC2080 Temperature and Humidity Sensor.
*/
#include "hdc2080.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

/* TWI instance. */
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static uint8_t hdc2080_address;

static void open_reg(uint8_t reg) {
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(&m_twi, hdc2080_address, &reg, sizeof(reg),
                            false); // point to specified register
  APP_ERROR_CHECK(err_code);
}

static uint8_t read_reg(uint8_t reg) {
  ret_code_t err_code;
  open_reg(reg);
  uint8_t reading; // holds byte of read data
  err_code = nrf_drv_twi_rx(&m_twi, hdc2080_address, &reading, sizeof(reading));
  APP_ERROR_CHECK(err_code);
  return reading;
}

static void write_reg(uint8_t reg, uint8_t data) {
  ret_code_t err_code;
  uint8_t send[2] = {reg, data};
  err_code = nrf_drv_twi_tx(&m_twi, hdc2080_address, send, sizeof(send),
                            false); // point to specified register
  APP_ERROR_CHECK(err_code);
}

void set_measurement_mode(int mode) {
  uint8_t config_contents;
  config_contents = read_reg(MEASUREMENT_CONFIG);
  switch (mode) {
  case TEMP_AND_HUMID:
    config_contents = (config_contents & 0xF9);
    break;
  case TEMP_ONLY:
    config_contents = (config_contents & 0xFC);
    config_contents = (config_contents | 0x02);
    break;
  case HUMID_ONLY:
    config_contents = (config_contents & 0xFD);
    config_contents = (config_contents | 0x04);
    break;
  default:
    config_contents = (config_contents & 0xF9);
  }
  write_reg(MEASUREMENT_CONFIG, config_contents);
}

/*Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
void set_temp_res(int resolution) {
  uint8_t config_contents;
  config_contents = read_reg(MEASUREMENT_CONFIG);
  switch (resolution) {
  case FOURTEEN_BIT:
    config_contents = (config_contents & 0x3F);
    break;
  case ELEVEN_BIT:
    config_contents = (config_contents & 0x7F);
    config_contents = (config_contents | 0x40);
    break;
  case NINE_BIT:
    config_contents = (config_contents & 0xBF);
    config_contents = (config_contents | 0x80);
    break;
  default:
    config_contents = (config_contents & 0x3F);
  }
  write_reg(MEASUREMENT_CONFIG, config_contents);
}

/*Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
        the humidity resolution*/
void set_humid_res(int resolution) {
  uint8_t config_contents;
  config_contents = read_reg(MEASUREMENT_CONFIG);
  switch (resolution) {
  case FOURTEEN_BIT:
    config_contents = (config_contents & 0xCF);
    break;
  case ELEVEN_BIT:
    config_contents = (config_contents & 0xDF);
    config_contents = (config_contents | 0x10);
    break;
  case NINE_BIT:
    config_contents = (config_contents & 0xEF);
    config_contents = (config_contents | 0x20);
    break;
  default:
    config_contents = (config_contents & 0xCF);
  }
  write_reg(MEASUREMENT_CONFIG, config_contents);
}

/*Bit 0 of the MEASUREMENT_CONFIG register can be used
        to trigger measurements  */
void trigger_measurement(void) {
  uint8_t config_contents;
  config_contents = read_reg(MEASUREMENT_CONFIG);
  config_contents = (config_contents | 0x01);
  write_reg(MEASUREMENT_CONFIG, config_contents);
}

void reset(void) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  config_contents = (config_contents | 0x80);
  write_reg(INT_CONFIG, config_contents);
  nrf_delay_ms(50);
}

float read_temp(void) {
  uint8_t byte[2];
  uint16_t temp;
  byte[0] = read_reg(TEMP_LOW);
  byte[1] = read_reg(TEMP_HIGH);
  temp = (unsigned int)byte[1] << 8 | byte[0];
  return (float)(temp) * 165 / 65536 - 40.5;
}

float read_humidity(void) {
  uint8_t byte[2];
  uint16_t humidity;
  byte[0] = read_reg(HUMID_LOW);
  byte[1] = read_reg(HUMID_HIGH);
  humidity = (unsigned int)byte[1] << 8 | byte[0];
  return (float)(humidity) / 65536 * 100;
}

/*Bit 2 of the INT_CONFIG register can be used to enable/disable
        the interrupt pin  */
void enable_interrupt(void) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  config_contents = (config_contents | 0x04);
  write_reg(INT_CONFIG, config_contents);
}

/*Bit 2 of the INT_CONFIG register can be used to enable/disable
        the interrupt pin  */
void disable_interrupt(void) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  config_contents = (config_contents & 0xFB);
  write_reg(INT_CONFIG, config_contents);
}

/*Bits 6-4  of the INT_CONFIG register controls the measurement
        rate  */
void set_rate(int rate) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  switch (rate) {
  case MANUAL:
    config_contents = (config_contents & 0x8F);
    break;
  case TWO_MINS:
    config_contents = (config_contents & 0x9F);
    config_contents = (config_contents | 0x10);
    break;
  case ONE_MINS:
    config_contents = (config_contents & 0xAF);
    config_contents = (config_contents | 0x20);
    break;
  case TEN_SECONDS:
    config_contents = (config_contents & 0xBF);
    config_contents = (config_contents | 0x30);
    break;
  case FIVE_SECONDS:
    config_contents = (config_contents & 0xCF);
    config_contents = (config_contents | 0x40);
    break;
  case ONE_HZ:
    config_contents = (config_contents & 0xDF);
    config_contents = (config_contents | 0x50);
    break;
  case TWO_HZ:
    config_contents = (config_contents & 0xEF);
    config_contents = (config_contents | 0x60);
    break;
  case FIVE_HZ:
    config_contents = (config_contents | 0x70);
    break;
  default:
    config_contents = (config_contents & 0x8F);
  }
  write_reg(INT_CONFIG, config_contents);
}

/*Bit 1 of the INT_CONFIG register can be used to control the
        the interrupt pins polarity */
void set_interrupt_polarity(int polarity) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  switch (polarity) {
  case ACTIVE_LOW:
    config_contents = (config_contents & 0xFD);
    break;
  case ACTIVE_HIGH:
    config_contents = (config_contents | 0x02);
    break;
  default:
    config_contents = (config_contents & 0xFD);
  }
  write_reg(INT_CONFIG, config_contents);
}

/* Bit 0 of the INT_CONFIG register can be used to control the
   interrupt pin's mode */
void set_interrupt_mode(int mode) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  switch (mode) {
  case LEVEL_MODE:
    config_contents = (config_contents & 0xFE);
    break;
  case COMPARATOR_MODE:
    config_contents = (config_contents | 0x01);
    break;
  default:
    config_contents = (config_contents & 0xFE);
  }
  write_reg(INT_CONFIG, config_contents);
}

uint8_t read_interrupt_status(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(INTERRUPT_DRDY);
  return reg_contents;
}

// Enables the interrupt pin for comfort zone operation
void enable_threshold_interrupt(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(INTERRUPT_CONFIG);
  reg_contents = (reg_contents | 0x78);
  write_reg(INTERRUPT_CONFIG, reg_contents);
}

// Disables the interrupt pin for comfort zone operation
void disable_threshold_interrupt(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(INTERRUPT_CONFIG);
  reg_contents = (reg_contents & 0x87);
  write_reg(INTERRUPT_CONFIG, reg_contents);
}

// enables the interrupt pin for DRDY operation

void enable_drdy_interrupt(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(INTERRUPT_CONFIG);
  reg_contents = (reg_contents | 0x80);
  write_reg(INTERRUPT_CONFIG, reg_contents);
}

// disables the interrupt pin for DRDY operation
void disable_drdy_interrupt(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(INTERRUPT_CONFIG);
  reg_contents = (reg_contents & 0x7F);
  write_reg(INTERRUPT_CONFIG, reg_contents);
}

void enable_heater(void) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  config_contents = (config_contents | 0x08);
  write_reg(INT_CONFIG, config_contents);
}

void disable_heater(void) {
  uint8_t config_contents;
  config_contents = read_reg(INT_CONFIG);
  config_contents = (config_contents & 0xF7);
  write_reg(INT_CONFIG, config_contents);
}

void set_low_temp(float temp) {
  uint8_t temp_thresh_low;
  if (temp < -40) {
    temp = -40;
  } else if (temp > 125) {
    temp = 125;
  }
  // Calculate value to load into register
  temp_thresh_low = (uint8_t)(256 * (temp + 40.5) / 165);
  write_reg(TEMP_THR_L, temp_thresh_low);
}

void set_high_temp(float temp) {
  uint8_t temp_thresh_high;
  // Verify user is not trying to set value outside bounds

  if (temp < -40) {
    temp = -40;
  } else if (temp > 125) {
    temp = 125;
  }
  // Calculate value to load into register
  temp_thresh_high = (uint8_t)(256 * (temp + 40.5) / 165);
  write_reg(TEMP_THR_H, temp_thresh_high);
}

void set_high_humidity(float humid) {
  uint8_t humid_thresh;
  // Verify user is not trying to set value outside bounds

  if (humid < 0) {
    humid = 0;
  } else if (humid > 100) {
    humid = 100;
  }
  // Calculate value to load into register
  humid_thresh = (uint8_t)(256 * (humid) / 100);
  write_reg(HUMID_THR_H, humid_thresh);
}

void set_low_humidity(float humid) {
  uint8_t humid_thresh;
  // Verify user is not trying to set value outside bounds
  if (humid < 0) {
    humid = 0;
  } else if (humid > 100) {
    humid = 100;
  }
  // Calculate value to load into register
  humid_thresh = (uint8_t)(256 * (humid) / 100);
  write_reg(HUMID_THR_L, humid_thresh);
}

//  Return humidity from the low threshold register
float read_low_humidity_threshold(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(HUMID_THR_L);
  return (float)reg_contents * 100 / 256;
}

//  Return humidity from the high threshold register
float read_high_humidity_threshold(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(HUMID_THR_H);
  return (float)reg_contents * 100 / 256;
}

//  Return temperature from the low threshold register
float read_low_temp_threshold(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(TEMP_THR_L);
  return (float)reg_contents * 165 / 256 - 40;
}

//  Return temperature from the high threshold register
float read_high_temp_threshold(void) {
  uint8_t reg_contents;
  reg_contents = read_reg(TEMP_THR_H);
  return (float)reg_contents * 165 / 256 - 40;
}

uint8_t read_temp_offset_adjust(void) { return read_reg(TEMP_OFFSET_ADJUST); }

uint8_t set_temp_offset_adjust(uint8_t value) {
  write_reg(TEMP_OFFSET_ADJUST, value);
  return read_temp_offset_adjust();
}

uint8_t read_humidity_offset_adjust(void) {
  return read_reg(HUM_OFFSET_ADJUST);
}

uint8_t set_humidity_offset_adjust(uint8_t value) {
  write_reg(HUM_OFFSET_ADJUST, value);
  return read_humidity_offset_adjust();
}

uint16_t read_manufacturer_id(void) {
  uint8_t byte[2];
  byte[0] = read_reg(MID_L);
  byte[1] = read_reg(MID_H);
  return (byte[1] << 8) | byte[0];
}

uint16_t read_device_id(void) {
  uint8_t byte[2];
  byte[0] = read_reg(DEVICE_ID_L);
  byte[1] = read_reg(DEVICE_ID_H);
  return (byte[1] << 8) | byte[0];
}

void hdc2080_init(uint8_t hw_address) { hdc2080_address = hw_address; }