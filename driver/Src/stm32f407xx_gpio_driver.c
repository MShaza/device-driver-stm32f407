/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Aug 24, 2024
 *      Author: muhammadshazab
 */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn					GPIO CLOCK CONTROL
 * @brief				Disable or enable peripheral clock
 * @Param				Peripheral base address, Enable or Disable parameter(0,1)
 * @Return				None
 * @note				None
 * */

void GPIO_Clock_Control(GPIO_RefDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi ==ENABLE){
			if(pGPIOx == GPIOA){
				GPIOA_PCLK_EN()	;
			}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_EN();
				}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOC){
					GPIOC_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOD){
					GPIOD_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOE){
					GPIOE_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOF){
					GPIOF_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOG){
					GPIOG_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOH){
					GPIOH_PCLK_EN()	;
				}
				else if(pGPIOx == GPIOI){
					GPIOI_PCLK_EN()	;
				}
		}
		else
		{
			if(pGPIOx == GPIOA){
				GPIOA_PCLK_DI();
			}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOC){
					GPIOC_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOD){
					GPIOD_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOE){
					GPIOE_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOF){
					GPIOF_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOG){
					GPIOG_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOH){
					GPIOH_PCLK_DI()	;
				}
				else if(pGPIOx == GPIOI){
					GPIOI_PCLK_DI()	;
				}
	}

}

/************************************************************************************
 * @fn					GPIO Mode Initialisation
 * @brief				Set the mode the the GPIO PIN
 * @Param				Mode of the pin is given
 * @Return				None
 * @note				None
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp =0;
			if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode <=GPIOx_PIN_SET_ALT_FN){
				//set the mode of the GPIO PIN

				temp = pGPIOHandle->GPIO_Config_t.GPIO_PinMode <<(2*pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
				pGPIOHandle->pGPIOx->MODER = temp;
			}
			else{
				//todo
			}
			//set the speed of the GPIO Pin
			temp = 0;
			temp =  pGPIOHandle->GPIO_Config_t.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OSPEEDR |= temp;

			//Set GPIO Pin Pull Up and Pull Down Control
			temp = 0;
			temp =  pGPIOHandle->GPIO_Config_t.GPIO_PinPuPdCntrl <<(2*pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->PUPDR |= temp;

			//Set GPIO Pin Pull Up and Pull Down Control
			temp = 0;
			temp =  pGPIOHandle->GPIO_Config_t.GPIO_PinOPType <<(pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OTYPER |= temp;

			if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode == GPIOx_PIN_SET_ALT_FN){
				temp = 0;
				uint32_t temp1 = 0;
				uint32_t temp2 = 0;
				temp1 = pGPIOHandle->GPIO_Config_t.GPIO_PinNumber / 8;
				temp2 = pGPIOHandle->GPIO_Config_t.GPIO_PinNumber % 8;
				temp = pGPIOHandle->GPIO_Config_t.GPIO_PinAltFunMode << (4*temp2);
				pGPIOHandle->pGPIOx->AFRL[temp1] &= ~(0xF << 4*temp2);
				pGPIOHandle->pGPIOx->AFRL[temp1] |= temp;
			}

}

/*********************************************************************
 * @fn					GPIO Read input Pin
 * @brief				Read the value form the Input pin
 * @Param				Peripheral base address, Pin Number
 * @Return				uint8_t values present on pin
 * @note				None
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;

}
/*********************************************************************
 * @fn					GPIO Read input Port
 * @brief				Read the value form the Input Port
 * @Param				Peripheral base address
 * @Return				uint16_t values present on Port
 * @note				None
 * */
uint16_t GPIO_ReadFromOutputPort(GPIO_RefDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}
/*********************************************************************
 * @fn					GPIO_WriteToOutputPin
 * @brief				Write value to Output Pin
 * @Param				Peripheral base address, Pin Number, value
 * @Return				None
 * @note				None
 * */
void GPIO_WriteToOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR |= ~(1 << PinNumber);
	}
}
/*********************************************************************
 * @fn					GPIO_WriteToOutputPort
 * @brief				Write value to Output Port
 * @Param				Peripheral base address, value
 * @Return				None
 * @note				None
 * */
void GPIO_WriteToOutputPort(GPIO_RefDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

