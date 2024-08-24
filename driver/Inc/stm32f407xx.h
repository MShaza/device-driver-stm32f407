/*
 * stm32f407xx.h
 *
 *  Created on: Aug 24, 2024
 *      Author: muhammadshazab
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __v volatile

#define FLASH_BASE_ADDR 				0x08000000U 		// Base address of flash memory
#define SRAM1_BASE_ADDR					0x20000000U			// Base address of SRAM1
#define SRAM_BASE_ADDR					SRAM1_ADDR
#define SRAM2_BASE_ADDR					0x2001C000U			// Base address of SRAM3
#define ROM_BASE_ADDR					0x1FFF0000U			// Base address of ROM

#define PERIPH_BASE_ADDR				0x40000000U			// Starting address of Peripherals
#define APB1_PERIPH_BASE_ADDR			PERIPH_BASE_ADDR
#define	APB2_PERIPH_BASE_ADDR			0x40010000U			// Base Address of APB2 Bus
#define AHB1_PERIPH_BASE_ADDR			0x40020000U			// Base Address of AHB1 Bus
#define AHB2_PERIPH_BASE_ADDR			0x50000000U			// Base Address of AHB2 BUs
#define RCC_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x3800)

/********** BASE ADDRESS OF ALL THE PERIPHERAL HANIGING TO AHB1 BUS ***************/
#define GPIOA_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x2000)
#define GPIOJ_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x2400)
#define GPIOK_BASE_ADDR					(AHB1_PERIPH_BASE_ADDR + 0x2800)

/********** BASE ADDRESS OF ALL THE PERIPHERAL HANIGING TO APB1 BUS ***************/
#define I2C1_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x5C00)
#define SPI2_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x3C00)
#define USART2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR					(APB1_PERIPH_BASE_ADDR + 0x5000)

/********** BASE ADDRESS OF ALL THE PERIPHERAL HANIGING TO APB2 BUS ***************/
#define SPI1_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x3000)
#define USART1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x1400)
#define EXT1_BASE_ADDR					(APB2_PERIPH_BASE_ADDR + 0x3C00)
#define SYSCFG_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3800)

/*
 * Define type struct for the GPIO registers
 * */
typedef struct {
	__v uint32_t MODER;				// GPIO MODE Register
	__v uint32_t OTYPER;			// GPIO Output type Register
	__v uint32_t OSPEEDR;			// GPIO Output speed register
	__v uint32_t PUPDR;				// GPIO Pull up/Pull Down register
	__v uint32_t IDR;				// GPIO Input Data Register
	__v uint32_t ODR;				// GPIO Output Data Register
	__v uint32_t OBSRR;				// GPIO Bit Set/Reset Register
	__v uint32_t LCKR;				// GPIO Configuration Lock Register
	__v uint32_t AFRL[2];			// AFRL[0] - Alternate function low register
									// AFRL[1] - Alternate function high register
}GPIO_RefDef_t;

typedef struct {
	__v uint32_t CR;				// RCC clock control register
	__v uint32_t PLLCFGR;			// RCC PLL configuration register
	__v uint32_t CFGR;				// RCC clock configuration register
	__v uint32_t CIR;				// RCC clock interrupt register
	__v uint32_t AHB1RSTR;			// RCC AHB1 Peripheral reset register
	__v uint32_t AHB2RSTR;			// RCC AHB2 Peripheral reset register
	__v uint32_t AHB3RSTR;			// RCC AHB3 Peripheral reset register
	 uint32_t Reserved0;
	__v uint32_t APB1RSTR;			// RCC APB1 Peripheral reset register
	__v uint32_t APB2RSTR;			// RCC APB2 Peripheral reset register
	 uint32_t Reserved1;
	 uint32_t Reserved2;
	__v uint32_t AHB1ENR;			// RCC AHB1 peripheral clock enable register
	__v uint32_t AHB2ENR;			// RCC AHB2 peripheral clock enable register
	__v uint32_t AHB3ENR;			// RCC AHB3 peripheral clock enable register
	 uint32_t Reserved3;
	__v uint32_t APB1ENR;			// RCC APB1 peripheral clock enable register
	__v uint32_t APB2ENR;			// RCC APB2 peripheral clock enable register
	 uint32_t Reserved4;
	 uint32_t Reserved5;
	__v uint32_t AHB1LPENR;			// RCC AHB1 peripheral clock enable in low power mode register
	__v uint32_t AHB2LPENR;			// RCC AHB2 peripheral clock enable in low power mode register
	__v uint32_t AHB3LPENR;			// RCC AHB3 peripheral clock enable in low power mode register
	 uint32_t Reserved6;
	 __v uint32_t APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register
	__v uint32_t APB2LPENR;			// RCC APB2 peripheral clock enable in low power mode register
	 uint32_t Reserved7;
	 uint32_t Reserved8;
	__v uint32_t BDCR;				// RCC Backup domain control register
	__v uint32_t CSR;				// CC clock control & status register
	 uint32_t Reserved9;
	 uint32_t Reserved10;
	__v uint32_t SSCGR;				// RCC spread spectrum clock generation register
	 uint32_t Reserved11;
	__v uint32_t PLLI2SCFGR;		// RCC PLLI2S configuration register


}RCC_RefDef_t;

/**
 * Type cast GPIO Peripheral to the struct type
 *
 * */
#define GPIOA 							((GPIO_RefDef_t*)GPIOA_BASE_ADDR)
#define GPIOB 							((GPIO_RefDef_t*)GPIOB_BASE_ADDR)
#define GPIOC 							((GPIO_RefDef_t*)GPIOC_BASE_ADDR)
#define GPIOD 							((GPIO_RefDef_t*)GPIOD_BASE_ADDR)
#define GPIOE 							((GPIO_RefDef_t*)GPIOE_BASE_ADDR)
#define GPIOF 							((GPIO_RefDef_t*)GPIOF_BASE_ADDR)
#define GPIOG 							((GPIO_RefDef_t*)GPIOG_BASE_ADDR)
#define GPIOH 							((GPIO_RefDef_t*)GPIOH_BASE_ADDR)
#define GPIOI 							((GPIO_RefDef_t*)GPIOI_BASE_ADDR)
#define GPIOJ 							((GPIO_RefDef_t*)GPIOJ_BASE_ADDR)
#define GPIOK 							((GPIO_RefDef_t*)GPIOK_BASE_ADDR)

#define RCC								((RCC_RefDef_t*)RCC_BASE_ADDR)

/*
 * Enable clock for the GPIO PIN
 * */
#define GPIOA_PCLK_EN()						RCC->AHB1ENR |= (1<<0)
#define GPIOB_PCLK_EN()						RCC->AHB1ENR |= (1<<1)
#define GPIOC_PCLK_EN()						RCC->AHB1ENR |= (1<<2)
#define GPIOD_PCLK_EN()						RCC->AHB1ENR |= (1<<3)
#define GPIOE_PCLK_EN()						RCC->AHB1ENR |= (1<<4)
#define GPIOF_PCLK_EN()						RCC->AHB1ENR |= (1<<5)
#define GPIOG_PCLK_EN()						RCC->AHB1ENR |= (1<<6)
#define GPIOH_PCLK_EN()						RCC->AHB1ENR |= (1<<7)
#define GPIOI_PCLK_EN()						RCC->AHB1ENR |= (1<<8)

/*
 * Enable clock for the I2C PIN
 * */
#define I2C1_PCLK_EN()						RCC->APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()						RCC->APB1ENR |= (1<<22)
#define I2C3_PCLK_EN()						RCC->APB1ENR |= (1<<23)

/*
 * Enable clock for the SPI PIN
 * */
#define SPI1_PCLK_EN()						RCC->APB2ENR |= (1<<12)
#define SPI2_PCLK_EN()						RCC->APB1ENR |= (1<<14)
#define SPI3_PCLK_EN()						RCC->APB1ENR |= (1<<15)

/*
 * Enable clock for the USART/UART PIN
 * */
#define USART1_PCLK_EN()					RCC->APB2ENR |= (1<<4)
#define USART2_PCLK_EN()					RCC->APB1ENR |= (1<<17)
#define USART3_PCLK_EN()					RCC->APB1ENR |= (1<<18)
#define UART4_PCLK_EN()						RCC->APB1ENR |= (1<<19)
#define UART5_PCLK_EN()						RCC->APB1ENR |= (1<<20)
#define USART6_PCLK_EN()					RCC->APB2ENR |= (1<<5)

/*
 * Enable clock for the SYS_CFG PIN
 * */
#define SYS_CFG_PCLK_EN()					RCC->APB2ENR |= (1<<14)


/*
 * Disable clock for the GPIO PIN
 * */
#define GPIOA_PCLK_DI()						RCC->AHB1ENR &= ~(1<<0)
#define GPIOB_PCLK_DI()						RCC->AHB1ENR &= ~(1<<1)
#define GPIOC_PCLK_DI()						RCC->AHB1ENR &= ~(1<<2)
#define GPIOD_PCLK_DI()						RCC->AHB1ENR &= ~(1<<3)
#define GPIOE_PCLK_DI()						RCC->AHB1ENR &= ~(1<<4)
#define GPIOF_PCLK_DI()						RCC->AHB1ENR &= ~(1<<5)
#define GPIOG_PCLK_DI()						RCC->AHB1ENR &= ~(1<<6)
#define GPIOH_PCLK_DI()						RCC->AHB1ENR &= ~(1<<7)
#define GPIOI_PCLK_DI()						RCC->AHB1ENR &= ~(1<<8)

/*
 * Disable clock for the I2C PIN
 * */
#define I2C1_PCLK_DI()						RCC->APB1ENR &= ~(1<<21)
#define I2C2_PCLK_DI()						RCC->APB1ENR &= ~(1<<22)
#define I2C3_PCLK_DI()						RCC->APB1ENR &= ~(1<<23)

/*
 * Disable clock for the SPI PIN
 * */
#define SPI1_PCLK_DI()						RCC->APB2ENR &= ~(1<<12)
#define SPI2_PCLK_DI()						RCC->APB1ENR &= ~(1<<14)
#define SPI3_PCLK_DI()						RCC->APB1ENR &= ~(1<<15)

/*
 * Disable clock for the USART/UART PIN
 * */
#define USART1_PCLK_DI()					RCC->APB2ENR &= ~(1<<4)
#define USART2_PCLK_DI()					RCC->APB1ENR &= ~(1<<17)
#define USART3_PCLK_DI()					RCC->APB1ENR &= ~(1<<18)
#define UART4_PCLK_DI()						RCC->APB1ENR &= ~(1<<19)
#define UART5_PCLK_DI()						RCC->APB1ENR &= ~(1<<20)
#define USART6_PCLK_DI()					RCC->APB2ENR &= ~(1<<5)

/*
 * Disable clock for the SYS_CFG PIN
 * */
#define SYS_CFG_PCLK_DI()					RCC->APB2ENR &= ~(1<<14)

//Some basic Macros

#define ENABLE								1
#define DISABLE								0





#endif /* INC_STM32F407XX_H_ */



#endif /* INC_STM32F407XX_H_ */

