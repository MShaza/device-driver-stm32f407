#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx.h"
void SPI_Clock_Control(SPI_RefDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI1){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI1){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI1){
			SPI4_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI1){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI1){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI1){
			SPI4_PCLK_DI();
		}
	}
}
void SPI_init(SPI_Handle_t *pSPIHandle){
	uint32_t tempReg = 0;
	// set Slave and Master mode
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	// set SPI half and full duplex
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_FP_MODE){
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_HD_MODE){
		tempReg  &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_SIMP_RXONLY){
		tempReg |= (1 << SPI_CR1_BIDIMODE);
		tempReg |= (1 << SPI_CR1_RXONLY);
	}
	// set the data frame format
		tempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	// set the Software salve management
		tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
		// set the value for clock  polarity
		tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
		// set the clock phase
		tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
		pSPIHandle->pSPIx->SPI_CR1 = tempReg;
}
void sendData(SPI_RefDef_t *pSPIx, uint8_t *dataBuffer, uint32_t bufferLen){
	// check if TX flag is set
	while(checkSPIFlag(pSPIx,SPI_TX_FLAG) == FLAG_RESET);
	// check the data format frame
	if((pSPIx->SPI_CR1) & (1 << SPI_CR1_DFF)){
		// 16 bit data frame format
		pSPIx->SPI_DR = *((uint16_t*)dataBuffer);
		bufferLen -= 2;
	}
	else{
		// 8 bit data frame format
		pSPIx->SPI_DR = *((uint8_t*)dataBuffer);
		bufferLen--;
	}
}
uint8_t checkSPIFlag(SPI_RefDef_t *pSPIx, uint32_t flagName){
	if(pSPIx->SPI_SR & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
