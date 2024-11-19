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
			if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode <=GPIOx_PIN_SET_ANG){
				//set the mode of the GPIO PIN

				temp = pGPIOHandle->GPIO_Config_t.GPIO_PinMode <<(2*pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
				pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
				pGPIOHandle->pGPIOx->MODER |= temp;
			}
			else{
				// Interrupt configuration for the pins
				if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode == GPIOx_PIN_SET_IT_FT){
					//Setting the the pin as falling trigger
						EXTI->FTSR |= (1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
						//Reset the the pin as Rising trigger
						EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);

				}
				else if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode == GPIOx_PIN_SET_IT_RT){
					//Setting the the pin as falling trigger
					EXTI->RTSR |= (1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
					//Reset the the pin as Falling trigger
					EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);

				}
				else if(pGPIOHandle->GPIO_Config_t.GPIO_PinMode == GPIOx_PIN_SET_IT_RFT){
					//Setting the the pin as falling trigger
					EXTI->RTSR |= (1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
					//Setting the the pin as Rising trigger
					EXTI->FTSR |= (1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);
					}
				//  configure the port on sysconfig register

				uint8_t temp1 =  pGPIOHandle->GPIO_Config_t.GPIO_PinNumber / 4; // identify EXTRC register
				uint8_t temp2 = pGPIOHandle->GPIO_Config_t.GPIO_PinNumber % 4; // identify block to use in EXTICR
				uint8_t portCode = GPIO_BASE_ADDR_TO_PORT_CODE(pGPIOHandle->pGPIOx);
				SYS_CFG_PCLK_EN();
				SYSCFG->SYSCFG_EXTICR[temp1] = portCode << (temp2 *4);

				//configuring the IMR Register
				EXTI->IMR |= (1<<pGPIOHandle->GPIO_Config_t.GPIO_PinNumber);

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
/*********************************************************************
 * @fn					GPIO_ToggleOutputPin
 * @brief				Toggle output Pin
 * @Param				Peripheral base address, Pin Number
 * @Return				None
 * @note				None
 * */
void GPIO_ToggleOutputPin(GPIO_RefDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR = (1 << PinNumber);

}
/*********************************************************************
 * @fn					GPIO_IRQConfig
 * @brief				ENABLE OF DIABLE INTERRUPT ON NVIC
 * @Param				IRQ Number, ENable or disable Interrupt, priority of interrupt
 * @Return				None
 * @note				None
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << IRQNumber % 64 );
		}
	}
	else {
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
				}
				else if(IRQNumber > 31 && IRQNumber < 64){
					*NVIC_ICER0 |= (1 << IRQNumber % 32);
				}
				else if(IRQNumber >= 64 && IRQNumber < 96){
					*NVIC_ICER0 |= (1 << IRQNumber % 96);
				}
	}

}

/*********************************************************************
 * @fn					GPIO_IRQPriorityConfig
 * @brief				Set priority for the interrupt
 * @Param				IRQ Number,priority of interrupt
 * @Return				None
 * @note				None
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t Interrupt_Priority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount =  (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTATION);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (Interrupt_Priority << shift_amount);

}
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1 << PinNumber)){
		EXTI->PR |= (1 << PinNumber);
	}
}

