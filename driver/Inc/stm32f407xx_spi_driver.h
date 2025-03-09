#ifndef INC_STDM32F407XX_SPI_DRIVER_H_
#define INC_STDM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;
typedef struct{
	SPI_RefDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;
void SPI_Clock_Control(SPI_RefDef_t *pSPIx, uint8_t EnorDi);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void sendData(SPI_RefDef_t *pSPIx, uint8_t* dataBuffer, uint32_t bufferLen);
uint8_t checkSPIFlag(SPI_RefDef_t *pSPIx, uint32_t flagName);

/*********************************************1**********************************************
 * 					SOME BASIC MACROS OF SPI REGISTERS
 * *******************************************************************************************/

// SPI CRC1 REGISTER

#define SPI_MSTR_CONFIG			1			// MASTER SET
#define	SPI_SLAVE_CONFIG		0			// SLAVE SET
#define SPI_FP_MODE  			0			// SET FULL DUPLEX
#define SPI_HD_MODE				1			// SET HALF DUPLEX
#define SPI_SIMP_RXONLY			3			// SET RECIEVE ONLY MODE

// SPI RC REGISTER
#define SPI_SR_BSY				7			// BUSY FALG
#define SPI_SR_TXE				1			// TRANSMIT BUFFER EMPTY
#define SPI_RC_RXNE				0			// RECIEVE BUFFER NOT EMPTY
/*******************************************************************************************
 * 					SOME BASIC MACROS
 * *******************************************************************************************/
#define SPI_TX_FLAG				(1 << SPI_SR_TXE)

#endif
