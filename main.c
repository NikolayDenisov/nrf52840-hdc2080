#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "hdc2080.h"
#include "nrf_delay.h"

/*TWI instance ID. */
#
if TWI0_ENABLED
#define TWI_INSTANCE_ID 0# elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1# endif

/*Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

/*TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 *@brief TWI initialization.
 */
void twi_init(void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config = { .scl = ARDUINO_SCL_PIN,
		.sda = ARDUINO_SDA_PIN,
		.frequency = NRF_DRV_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init = false
	};

	err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_twi_enable(&m_twi);
}

void sensor_init(uint8_t *addr)
{
	// Begin with a device reset
	hdc2080_init(*addr);
	reset();
	enableInterrupt();
	enableDRDYInterrupt();
	setInterruptPolarity(ACTIVE_HIGH);
	setInterruptMode(LEVEL_MODE);
	setRate(FIVE_SECONDS);

	setMeasurementMode(TEMP_AND_HUMID);
	// Configure Measurements
	setTempRes(FOURTEEN_BIT);
	setHumidRes(FOURTEEN_BIT);
	nrf_delay_ms(130);
}

uint8_t detect_address()
{
	ret_code_t err_code;
	uint8_t address = 0;
	uint8_t sample_data;
	bool detected_device = false;
	
	while (!detected_device && address <= TWI_ADDRESSES)
	{
		address++;
		err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
		if (err_code == NRF_SUCCESS)
		{
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
int main(void)
{
	uint8_t address = 0;
	uint8_t data_ready = 0x80;
	uint8_t status_int_register;

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	NRF_LOG_INFO("\r\nTWI sensor example started.");
	NRF_LOG_FLUSH();

	twi_init();
	address = detect_address();
	sensor_init(&address);
	// Begin measuring
	triggerMeasurement();
	while (true)
	{
		status_int_register = readInterruptStatus();
		if (status_int_register != data_ready)
		{
			nrf_delay_ms(500);
			continue;
		}

		NRF_LOG_INFO("Temperature: "
			NRF_LOG_FLOAT_MARKER " C.", NRF_LOG_FLOAT(readTemp()));
		NRF_LOG_INFO("Humidity: "
			NRF_LOG_FLOAT_MARKER "%%.", NRF_LOG_FLOAT(readHumidity()));
		NRF_LOG_FLUSH();
	}
}

/**@} */
