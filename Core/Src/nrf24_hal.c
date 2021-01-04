#include "nrf24_hal.h"
#include "stm32l4xx_hal_spi_ex.h"

SPI_HandleTypeDef *nRF24_SPI = NULL;

// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) {
    // Configure CSN pin
	nRF24_CSN_H();
	nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	uint8_t ret;
	HAL_StatusTypeDef status;

	nRF24_CSN_L();
	status = HAL_SPI_Transmit(nRF24_SPI_PORT, &data, 1, 100);
	if (status != HAL_OK) {
		return 0;
	}
	status = HAL_SPI_Receive(nRF24_SPI_PORT, &ret, 1, 100);
	if (status != HAL_OK) {
		return 0;
	}
	nRF24_CSN_H();

	return ret;
}
