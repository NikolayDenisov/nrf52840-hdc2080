#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "hdc2080.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdio.h>

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1
#endif

#define PIN_IN NRF_GPIO_PIN_MAP(0, 4)
#define PIN_OUT BSP_LED_0

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 *@brief TWI initialization.
 */
void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {.scl = ARDUINO_SCL_PIN,
                                           .sda = ARDUINO_SDA_PIN,
                                           .frequency = NRF_DRV_TWI_FREQ_100K,
                                           .interrupt_priority =
                                               APP_IRQ_PRIORITY_HIGH,
                                           .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  nrf_drv_gpiote_out_toggle(PIN_OUT);
}

/**
 *@brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 *and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

  err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}

void sensor_init(uint8_t *addr) {
  // Begin with a device reset
  hdc2080_init(*addr);
  reset();
  enable_interrupt();
  enable_drdy_interrupt();
  set_interrupt_polarity(ACTIVE_LOW);
  set_interrupt_mode(LEVEL_MODE);
  set_rate(TEN_SECONDS);

  set_measurement_mode(TEMP_AND_HUMID);
  // Configure Measurements
  set_temp_res(FOURTEEN_BIT);
  set_humid_res(FOURTEEN_BIT);
  set_high_temp(40);
  set_low_temp(-40);
  set_high_humidity(70);
  set_low_humidity(10);
  nrf_delay_ms(130);
  // Begin measuring
  trigger_measurement();
}

uint8_t detect_address() {
  ret_code_t err_code;
  uint8_t address = 0;
  uint8_t sample_data;
  bool detected_device = false;
  while (!detected_device && address <= TWI_ADDRESSES) {
    address++;
    err_code =
        nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
    if (err_code == NRF_SUCCESS) {
      detected_device = true;
      NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
      NRF_LOG_FLUSH();
    }
  }

  return address;
}

/**
 *@brief Function for main application entry.
 */
int main(void) {
  uint8_t address = 0;
  uint8_t data_ready = 0x80;
  uint8_t status_int_register;
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  NRF_LOG_INFO("\r\nTWI sensor example started.");
  NRF_LOG_FLUSH();
  gpio_init();
  twi_init();
  address = detect_address();
  sensor_init(&address);
  while (true) {
    status_int_register = read_interrupt_status();
    if (status_int_register != data_ready) {
      nrf_delay_ms(500);
      continue;
    }

    NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER " C.",
                 NRF_LOG_FLOAT(read_temp()));
    NRF_LOG_INFO("Humidity: " NRF_LOG_FLOAT_MARKER "%%.",
                 NRF_LOG_FLOAT(read_humidity()));
    NRF_LOG_FLUSH();
  }
}

/**@} */