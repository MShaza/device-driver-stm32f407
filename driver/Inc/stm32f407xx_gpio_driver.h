/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 28, 2024
 *      Author: muhammadshazab
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*************************************************************************************
 * Creating Macros for the GPIO Peripheral Register
 * */

//@ Set GPIO Mode Register
#define GPIOx_PIN_SET_IN 					0			//Set Input Mode
#define GPIOx_PIN_SET_OUT					1			//Set Output Mode
#define GPIOx_PIN_SET_ALT_FN				2			//Set Alternate Function
#define GPIOx_PIN_SET_ANG					3			//Set Pin as Analogue

//@ Set the GPIO Output Speed Register
#define GPIOx_PIN_OUT_LS					0			//Low Speed
#define GPIOx_PIN_OUT_MS					1			//Medium Speed
#define GPIOx_PIN_OUT_FS					2			//Fast Speed
#define GPIOx_PIN_OUT_VHS					3			//Very High Speed

//@ Set the GPIO Port Pull up and Pull Down Register
#define GPIOx_PIN_NO_PULL_UP				0
#define GPIOx_PIN_PULL_UP					1
#define GPIOx_PIN_PULL_DWN					2

//@ Set the GPIO Port Pull up and Pull Down Register
#define GPIOx_PIN_NO_PULL_UP				0
#define GPIOx_PIN_PULL_UP					1
#define GPIOx_PIN_PULL_DWN					2

//@ Set the GPIO Port Pull up and Pull Down Register
#define GPIOx_PIN_PSH_PULL					0
#define GPIOx_PIN_OPN_DRAIN					1

typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;					//@ Set GPIO Mode Register
	uint8_t GPIO_PinSpeed;					//@ Set GPIO Output speed Register
	uint8_t GPIO_PinPuPdCntrl;			 	//@ Set GPIO Pin Pull Up Or Pull Down Control
	uint8_t GPIO_PinOPType;					//@ Set the GPIO Port Type (Push-Pull, Open Drain )
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_RefDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_Config_t;

}GPIO_Handle_t;

void GPIO_Clock_Control(GPIO_RefDef_t *pGPIOx, uint8_t EnorDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RefDef_t *pGPIOx);
uint8_t GPIO_ReadFromInputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromOutputPort(GPIO_RefDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RefDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNUmber);
void GPIO_IRQConfig(uint8_t IRQNUmber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
