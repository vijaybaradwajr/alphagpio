/*
 * stm32f103c6xx.h
 *
 *  Created on: Jun 22, 2025
 *      Author: vijay
 */
#include"stdint.h"
#ifndef STM32F103C6XX_H_
#define STM32F103C6XX_H_

#define ENABLE 1
#define DISABLE 0



#define AHB_BASE_ADDR 0x40018000U
#define APB1_BASE_ADDR 0x40000000U
#define APB2_BASE_ADDR 0x40010000U


//Peripheral Address
#define GPIOA_BASE_ADDR (APB2_BASE_ADDR+(0X0800))
#define GPIOB_BASE_ADDR (APB2_BASE_ADDR+(0X0C00))
#define GPIOC_BASE_ADDR (APB2_BASE_ADDR+(0X1000))
#define GPIOD_BASE_ADDR (APB2_BASE_ADDR+(0X1400))
#define GPIOE_BASE_ADDR (APB2_BASE_ADDR+(0X1800))
#define GPIOF_BASE_ADDR (APB2_BASE_ADDR+(0X1C00))
#define GPIOG_BASE_ADDR (APB2_BASE_ADDR+(0X2000))
#define RCC_BASE_ADDR 	(AHB_BASE_ADDR+(0X8000))

///struct def for creating a register to load all those gpio values

typedef struct
{
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;


}GPIO_RegDef_t;

//GPIO Type
typedef struct
{
	uint32_t port;
	uint32_t pin;
	uint32_t mode;
	uint32_t pinmode;
	uint32_t pull;
	uint32_t speed;
	uint32_t alt_func;

}GPIO_PinConfig_t;
///struct def for registers to hold RCC values


typedef struct
{
	uint32_t CR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t AHBSTR;
	uint32_t CFGR2;
}RCC_RegDef_t;



//GPIO PIN NUMBERS AND POSITIONS




typedef struct{
	 uint32_t IMR;
	 uint32_t EMR;
	 uint32_t RTSR;
	 uint32_t FTSR;
	 uint32_t SWIER;
	 uint32_t PR;
}EXTI_RegDef_t;


typedef struct{
	 uint32_t EVCR;
	 uint32_t MAPR[3];
	 uint32_t EXTICR[4];
	 uint32_t MAPR2[2];
}AFIO_RegDef_t;


#define GPIO_PIN_NO_0      0
#define GPIO_PIN_NO_1      1
#define GPIO_PIN_NO_2      2
#define GPIO_PIN_NO_3      3
#define GPIO_PIN_NO_4      4
#define GPIO_PIN_NO_5      5
#define GPIO_PIN_NO_6      6
#define GPIO_PIN_NO_7      7
#define GPIO_PIN_NO_8      8
#define GPIO_PIN_NO_9      9
#define GPIO_PIN_NO_10     10
#define GPIO_PIN_NO_11     11
#define GPIO_PIN_NO_12     12
#define GPIO_PIN_NO_13     13
#define GPIO_PIN_NO_14     14
#define GPIO_PIN_NO_15     15

typedef enum{
 RISING_EDGE,
 FALLING_EDGE,
 RISING_FALLING_EDGE
}edge_select;
///points the registers to the addresses by using a pointer

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
//#define GPIOF (GPIO_RegDef_t*)GPIOF_BASE_ADDR
//#define GPIOG (GPIO_RegDef_t*)GPIOG_BASE_ADDR
#define RCC ((RCC_RegDef_t*)RCC_BASE_ADDR)



//points the registers to the addresses of RCC peripherals using a pointer
#define RCC_CR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x00))
#define RCC_CFGR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x04))
#define RCC_CIR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x08))
#define RCC_APB2RSTR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x0C))
#define RCC_APB1RSTR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x10))
#define RCC_AHBENR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x14))
#define RCC_APB2ENR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x18))
#define RCC_APB1ENR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x1C))
#define RCC_BDCR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x20))
#define RCC_CSR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x24))
#define RCC_AHBSTR (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x28))
#define RCC_CFGR2 (RCC_RegDef_t*)(RCC_BASE_ADDR+(0x2C))


//enable clocks for the same

#define GPIOA_Clk_En() (RCC->APB2ENR|=(1<<2))
#define GPIOB_Clk_En() (RCC->APB2ENR|=(1<<3))
#define GPIOC_Clk_En() (RCC->APB2ENR|=(1<<4))
#define GPIOD_Clk_En() (RCC->APB2ENR|=(1<<5))
#define GPIOE_Clk_En() (RCC->APB2ENR|=(1<<6))

#define GPIOA_Clk_Di() (RCC->APB2ENR&=~(1<<2))
#define GPIOB_Clk_Di() (RCC->APB2ENR&=~(1<<3))
#define GPIOC_Clk_Di() (RCC->APB2ENR&=~(1<<4))
#define GPIOD_Clk_Di() (RCC->APB2ENR&=~(1<<5))
#define GPIOE_Clk_Di() (RCC->APB2ENR&=~(1<<6))


/*GPIO PIN Modes
 * */
#define OUTPUT_MODE ((uint32_t)0x01)
#define INPUT_MODE  ((uint32_t)0x02)

//pin mode
#define GPIO_MODE_INPUT ((uint32_t)0x00)
#define GPIO_MODE_OUTPUT_10MHZ ((uint32_t)0x01)
#define GPIO_MODE_OUTPUT_2MHZ ((uint32_t)0x02)
#define GPIO_MODE_OUTPUT_50MHZ ((uint32_t)0x03)

//if input
#define GPIO_ANALOGMODE ((uint32_t)0x00)
#define GPIO_FLOATINGIN ((uint32_t)0x01)
#define GPIO_INPUPD ((uint32_t)0x02)
#define GPIO_RESERVED ((uint32_t)0x03)

//if output
#define GPIO_PUPD ((uint32_t)0x00)
#define GPIO_OPD ((uint32_t)0x01)
#define GPIO_AFPUPD ((uint32_t)0x02)
#define GPIO_AFUOPD ((uint32_t)0x03)

//pinpos
extern uint32_t PINPOS[16];
// some generic macros

#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_SET        1
#define FLAG_RESET      0

//interrupts
typedef enum
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
 NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
 HardFault_IRQn              = -13,    /*!< 3 Cortex-M3 Hard Fault Interrupt                     */
 MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
 BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
 UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
 SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
 DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
 PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
 SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  STM32 specific Interrupt Numbers *********************************************************/
 WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
 PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
 TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
 RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
 FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
 RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
 EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
 EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
 EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
 EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
 EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
 DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
 DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
 DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
 DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
 DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
 DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
 DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */
 ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
 USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
 USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
 CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
 CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
 EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
 TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
 TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
 TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
 TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
 TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
 TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
 I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
 I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
 SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
 USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
 USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
 EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
 RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
 USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
} IRQn_Type;


/***************************************  START: Processor Specific Details *********************************************/
/*
 *  ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0				(( uint32_t *)0xE000E100)
#define NVIC_ISER1				((uint32_t *)0xE000E104)
#define NVIC_ISER2				(( uint32_t *)0xE000E108)
#define NVIC_ISER3				(( uint32_t *)0xE000E10C)

/*
 *  ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0				(( uint32_t *)0XE000E180)
#define NVIC_ICER1				(( uint32_t *)0XE000E184)
#define NVIC_ICER2				(( uint32_t *)0XE000E188)
#define NVIC_ICER3				(( uint32_t *)0XE000E18C)


/*
 *  ARM Cortex Mx Processor NVIC Priority Register Addresses Calculation
 */

#define NVIC_PR_BASE_ADDR			(( uint32_t *) 0xE000E400)


#define NO_PR_BITS_IMPLEMENTED				4



/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR 					0x08000000U // flash memory base address
#define SRAM1_BASEADDR 					0x20000000U // SRAM base address, for stm32f1 there is only one SRM
#define ROM_BASEADDR 					0x1FFFF000U // ROM base address
#define SRAM 							SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base address
 */

#define PERIPH_BASEADDR 				0x40000000U
#define APB1PERIPH_BASEADDR             PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHBPERIPH_BASEADDR 				0x40018000U

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR   				(APB2PERIPH_BASEADDR + 0x0400)
#define AFIO_BASEADDR					(APB2PERIPH_BASEADDR)

#define GPIOA_BASEADDR  				(APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR  				(APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR  				(APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR  				(APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR  				(APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR  				(APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR  				(APB2PERIPH_BASEADDR + 0x2000)

#define SPI1_BASEADDR   				(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR 				(APB2PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define SPI2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5800)

#define bxCAN2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x6800)
#define bxCAN1_BASEADDR 				(APB1PERIPH_BASEADDR + 0x6400)




/*
 * Base addresses of peripherals which are hanging on AHB bus
 */

#define RCC_BASEADDR 					(AHBPERIPH_BASEADDR + 0x9000)



//#define EXTI0_IRQn 						6
//#define EXTI1_IRQn 						7
//#define EXTI2_IRQn 						8
//#define EXTI3_IRQn 						9
//#define EXTI4_IRQn 						10
//#define EXTI9_5_IRQn 					23
//#define EXTI15_10_IRQn 					40


/************************* Bit definition for AFIO_EXTICR1 register ***********************************/

#define AFIO_EXTICR1_EXTI0				((uint16_t) 0x000F);			/*!< EXTI 0 Configuration  */
#define AFIO_EXTICR1_EXTI1				((uint16_t) 0x00F0);			/*!< EXTI 1 Configuration  */
#define AFIO_EXTICR1_EXTI2				((uint16_t) 0x0F00);			/*!< EXTI 2 Configuration  */
#define AFIO_EXTICR1_EXTI3				((uint16_t) 0xF000);			/*!< EXTI 3 Configuration  */

/*!< EXTI0 Configuration  */
#define AFIO_EXTICR1_EXTI0_PA			((uint16_t) 0x0000);			/*!< PA[0] pin  */
#define AFIO_EXTICR1_EXTI0_PB			((uint16_t) 0x0001);			/*!< PB[0] pin  */
#define AFIO_EXTICR1_EXTI0_PC			((uint16_t) 0x0002);			/*!< PC[0] pin  */
#define AFIO_EXTICR1_EXTI0_PD			((uint16_t) 0x0003);			/*!< PD[0] pin  */
#define AFIO_EXTICR1_EXTI0_PE			((uint16_t) 0x0004);			/*!< PE[0] pin  */
#define AFIO_EXTICR1_EXTI0_PF			((uint16_t) 0x0005);			/*!< PF[0] pin  */
#define AFIO_EXTICR1_EXTI0_PG			((uint16_t) 0x0006);			/*!< PG[0] pin  */


/*!< EXTI1 Configuration  */
#define AFIO_EXTICR1_EXTI1_PA			((uint16_t) 0x0000);			/*!< PA[1] pin  */
#define AFIO_EXTICR1_EXTI1_PB			((uint16_t) 0x0010);			/*!< PB[1] pin  */
#define AFIO_EXTICR1_EXTI1_PC			((uint16_t) 0x0020);			/*!< PC[1] pin  */
#define AFIO_EXTICR1_EXTI1_PD			((uint16_t) 0x0030);			/*!< PD[1] pin  */
#define AFIO_EXTICR1_EXTI1_PE			((uint16_t) 0x0040);			/*!< PE[1] pin  */
#define AFIO_EXTICR1_EXTI1_PF			((uint16_t) 0x0050);			/*!< PF[1] pin  */
#define AFIO_EXTICR1_EXTI1_PG			((uint16_t) 0x0060);			/*!< PG[1] pin  */


/*!< EXTI2 Configuration  */
#define AFIO_EXTICR1_EXTI2_PA			((uint16_t) 0x0000);			/*!< PA[2] pin  */
#define AFIO_EXTICR1_EXTI2_PB			((uint16_t) 0x0100);			/*!< PB[2] pin  */
#define AFIO_EXTICR1_EXTI2_PC			((uint16_t) 0x0200);			/*!< PC[2] pin  */
#define AFIO_EXTICR1_EXTI2_PD			((uint16_t) 0x0300);			/*!< PD[2] pin  */
#define AFIO_EXTICR1_EXTI2_PE			((uint16_t) 0x0400);			/*!< PE[2] pin  */
#define AFIO_EXTICR1_EXTI2_PF			((uint16_t) 0x0500);			/*!< PF[2] pin  */
#define AFIO_EXTICR1_EXTI2_PG			((uint16_t) 0x0600);			/*!< PG[2] pin  */


/*!< EXTI3 Configuration  */
#define AFIO_EXTICR1_EXTI3_PA			((uint16_t) 0x0000);			/*!< PA[3] pin  */
#define AFIO_EXTICR1_EXTI3_PB			((uint16_t) 0x1000);			/*!< PB[3] pin  */
#define AFIO_EXTICR1_EXTI3_PC			((uint16_t) 0x2000);			/*!< PC[3] pin  */
#define AFIO_EXTICR1_EXTI3_PD			((uint16_t) 0x3000);			/*!< PD[3] pin  */
#define AFIO_EXTICR1_EXTI3_PE			((uint16_t) 0x4000);			/*!< PE[3] pin  */
#define AFIO_EXTICR1_EXTI3_PF			((uint16_t) 0x5000);			/*!< PF[3] pin  */
#define AFIO_EXTICR1_EXTI3_PG			((uint16_t) 0x6000);			/*!< PG[3] pin  */




/************************* Bit definition for AFIO_EXTICR2 register ***********************************/
#define AFIO_EXTICR2_EXTI4				((uint16_t) 0x000F);			/*!< EXTI 4 Configuration  */
#define AFIO_EXTICR2_EXTI5				((uint16_t) 0x00F0);			/*!< EXTI 5 Configuration  */
#define AFIO_EXTICR2_EXTI6				((uint16_t) 0x0F00);			/*!< EXTI 6 Configuration  */
#define AFIO_EXTICR2_EXTI7				((uint16_t) 0xF000);			/*!< EXTI 7 Configuration  */

/*!< EXTI4 Configuration  */
#define AFIO_EXTICR2_EXTI4_PA			((uint16_t) 0x0000);			/*!< PA[4] pin  */
#define AFIO_EXTICR2_EXTI4_PB			((uint16_t) 0x0001);			/*!< PB[4] pin  */
#define AFIO_EXTICR2_EXTI4_PC			((uint16_t) 0x0002);			/*!< PC[4] pin  */
#define AFIO_EXTICR2_EXTI4_PD			((uint16_t) 0x0003);			/*!< PD[4] pin  */
#define AFIO_EXTICR2_EXTI4_PE			((uint16_t) 0x0004);			/*!< PE[4] pin  */
#define AFIO_EXTICR2_EXTI4_PF			((uint16_t) 0x0005);			/*!< PF[4] pin  */
#define AFIO_EXTICR2_EXTI4_PG			((uint16_t) 0x0006);			/*!< PG[4] pin  */


/*!< EXTI1 Configuration  */
#define AFIO_EXTICR2_EXTI5_PA			((uint16_t) 0x0000);			/*!< PA[5] pin  */
#define AFIO_EXTICR2_EXTI5_PB			((uint16_t) 0x0010);			/*!< PB[5] pin  */
#define AFIO_EXTICR2_EXTI5_PC			((uint16_t) 0x0020);			/*!< PC[5] pin  */
#define AFIO_EXTICR2_EXTI5_PD			((uint16_t) 0x0030);			/*!< PD[5] pin  */
#define AFIO_EXTICR2_EXTI5_PE			((uint16_t) 0x0040);			/*!< PE[5] pin  */
#define AFIO_EXTICR2_EXTI5_PF			((uint16_t) 0x0050);			/*!< PF[5] pin  */
#define AFIO_EXTICR2_EXTI5_PG			((uint16_t) 0x0060);			/*!< PG[5] pin  */


/*!< EXTI2 Configuration  */
#define AFIO_EXTICR2_EXTI6_PA			((uint16_t) 0x0000);			/*!< PA[6] pin  */
#define AFIO_EXTICR2_EXTI6_PB			((uint16_t) 0x0100);			/*!< PB[6] pin  */
#define AFIO_EXTICR2_EXTI6_PC			((uint16_t) 0x0200);			/*!< PC[6] pin  */
#define AFIO_EXTICR2_EXTI6_PD			((uint16_t) 0x0300);			/*!< PD[6] pin  */
#define AFIO_EXTICR2_EXTI6_PE			((uint16_t) 0x0400);			/*!< PE[6] pin  */
#define AFIO_EXTICR2_EXTI6_PF			((uint16_t) 0x0500);			/*!< PF[6] pin  */
#define AFIO_EXTICR2_EXTI6_PG			((uint16_t) 0x0600);			/*!< PG[6] pin  */


/*!< EXTI3 Configuration  */
#define AFIO_EXTICR2_EXTI7_PA			((uint16_t) 0x0000);			/*!< PA[6] pin  */
#define AFIO_EXTICR2_EXTI7_PB			((uint16_t) 0x1000);			/*!< PB[6] pin  */
#define AFIO_EXTICR2_EXTI7_PC			((uint16_t) 0x2000);			/*!< PC[6] pin  */
#define AFIO_EXTICR2_EXTI7_PD			((uint16_t) 0x3000);			/*!< PD[6] pin  */
#define AFIO_EXTICR2_EXTI7_PE			((uint16_t) 0x4000);			/*!< PE[6] pin  */
#define AFIO_EXTICR2_EXTI7_PF			((uint16_t) 0x5000);			/*!< PF[6] pin  */
#define AFIO_EXTICR2_EXTI7_PG			((uint16_t) 0x6000);			/*!< PG[6] pin  */


/************************* Bit definition for AFIO_EXTICR3 register ***********************************/

#define AFIO_EXTICR3_EXTI8				((uint16_t) 0x000F);			/*!< EXTI 8 Configuration  */
#define AFIO_EXTICR3_EXTI9				((uint16_t) 0x00F0);			/*!< EXTI 9 Configuration  */
#define AFIO_EXTICR3_EXTI10				((uint16_t) 0x0F00);			/*!< EXTI 10 Configuration  */
#define AFIO_EXTICR4_EXTI11				((uint16_t) 0xF000);			/*!< EXTI 11 Configuration  */

/*!< EXTI4 Configuration  */
#define AFIO_EXTICR3_EXTI8_PA			((uint16_t) 0x0000);			/*!< PA[8] pin  */
#define AFIO_EXTICR3_EXTI8_PB			((uint16_t) 0x0001);			/*!< PB[8] pin  */
#define AFIO_EXTICR3_EXTI8_PC			((uint16_t) 0x0002);			/*!< PC[8] pin  */
#define AFIO_EXTICR3_EXTI8_PD			((uint16_t) 0x0003);			/*!< PD[8] pin  */
#define AFIO_EXTICR3_EXTI8_PE			((uint16_t) 0x0004);			/*!< PE[8] pin  */
#define AFIO_EXTICR3_EXTI8_PF			((uint16_t) 0x0005);			/*!< PF[8] pin  */
#define AFIO_EXTICR3_EXTI8_PG			((uint16_t) 0x0006);			/*!< PG[8] pin  */


/*!< EXTI1 Configuration  */
#define AFIO_EXTICR3_EXTI9_PA			((uint16_t) 0x0000);			/*!< PA[9] pin  */
#define AFIO_EXTICR3_EXTI9_PB			((uint16_t) 0x0010);			/*!< PB[9] pin  */
#define AFIO_EXTICR3_EXTI9_PC			((uint16_t) 0x0020);			/*!< PC[9] pin  */
#define AFIO_EXTICR3_EXTI9_PD			((uint16_t) 0x0030);			/*!< PD[9] pin  */
#define AFIO_EXTICR3_EXTI9_PE			((uint16_t) 0x0040);			/*!< PE[9] pin  */
#define AFIO_EXTICR3_EXTI9_PF			((uint16_t) 0x0050);			/*!< PF[9] pin  */
#define AFIO_EXTICR3_EXTI9_PG			((uint16_t) 0x0060);			/*!< PG[9] pin  */


/*!< EXTI2 Configuration  */
#define AFIO_EXTICR3_EXTI10_PA			((uint16_t) 0x0000);			/*!< PA[10] pin  */
#define AFIO_EXTICR3_EXTI10_PB			((uint16_t) 0x0100);			/*!< PB[10] pin  */
#define AFIO_EXTICR3_EXTI10_PC			((uint16_t) 0x0200);			/*!< PC[10] pin  */
#define AFIO_EXTICR3_EXTI10_PD			((uint16_t) 0x0300);			/*!< PD[10] pin  */
#define AFIO_EXTICR3_EXTI10_PE			((uint16_t) 0x0400);			/*!< PE[10] pin  */
#define AFIO_EXTICR3_EXTI10_PF			((uint16_t) 0x0500);			/*!< PF[10] pin  */
#define AFIO_EXTICR3_EXTI10_PG			((uint16_t) 0x0600);			/*!< PG[10] pin  */

/*!< EXTI3 Configuration  */
#define AFIO_EXTICR3_EXTI11_PA			((uint16_t) 0x0000);			/*!< PA[11] pin  */
#define AFIO_EXTICR3_EXTI11_PB			((uint16_t) 0x1000);			/*!< PB[11] pin  */
#define AFIO_EXTICR3_EXTI11_PC			((uint16_t) 0x2000);			/*!< PC[11] pin  */
#define AFIO_EXTICR3_EXTI11_PD			((uint16_t) 0x3000);			/*!< PD[11] pin  */
#define AFIO_EXTICR3_EXTI11_PE			((uint16_t) 0x4000);			/*!< PE[11] pin  */
#define AFIO_EXTICR3_EXTI11_PF			((uint16_t) 0x5000);			/*!< PF[11] pin  */
#define AFIO_EXTICR3_EXTI11_PG			((uint16_t) 0x6000);			/*!< PG[11] pin  */



/************************* Bit definition for AFIO_EXTICR4 register ***********************************/

#define AFIO_EXTICR4_EXTI12				((uint16_t) 0x000F);			/*!< EXTI 12 Configuration  */
#define AFIO_EXTICR4_EXTI13				((uint16_t) 0x00F0);			/*!< EXTI 13 Configuration  */
#define AFIO_EXTICR4_EXTI14				((uint16_t) 0x0F00);			/*!< EXTI 14 Configuration  */
#define AFIO_EXTICR4_EXTI15				((uint16_t) 0xF000);			/*!< EXTI 15 Configuration  */

/*!< EXTI4 Configuration  */
#define AFIO_EXTICR4_EXTI12_PA			((uint16_t) 0x0000);			/*!< PA[12] pin  */
#define AFIO_EXTICR4_EXTI12_PB			((uint16_t) 0x0001);			/*!< PB[12] pin  */
#define AFIO_EXTICR4_EXTI12_PC			((uint16_t) 0x0002);			/*!< PC[12] pin  */
#define AFIO_EXTICR4_EXTI12_PD			((uint16_t) 0x0003);			/*!< PD[12] pin  */
#define AFIO_EXTICR4_EXTI12_PE			((uint16_t) 0x0004);			/*!< PE[12] pin  */
#define AFIO_EXTICR4_EXTI12_PF			((uint16_t) 0x0005);			/*!< PF[12] pin  */
#define AFIO_EXTICR4_EXTI12_PG			((uint16_t) 0x0006);			/*!< PG[12] pin  */


/*!< EXTI1 Configuration  */
#define AFIO_EXTICR4_EXTI13_PA			((uint16_t) 0x0000);			/*!< PA[13] pin  */
#define AFIO_EXTICR4_EXTI13_PB			((uint16_t) 0x0010);			/*!< PB[13] pin  */
#define AFIO_EXTICR4_EXTI13_PC			((uint16_t) 0x0020);			/*!< PC[13] pin  */
#define AFIO_EXTICR4_EXTI13_PD			((uint16_t) 0x0030);			/*!< PD[13] pin  */
#define AFIO_EXTICR4_EXTI13_PE			((uint16_t) 0x0040);			/*!< PE[13] pin  */
#define AFIO_EXTICR4_EXTI13_PF			((uint16_t) 0x0050);			/*!< PF[13] pin  */
#define AFIO_EXTICR4_EXTI13_PG			((uint16_t) 0x0060);			/*!< PG[13] pin  */

/*!< EXTI2 Configuration  */
#define AFIO_EXTICR4_EXTI14_PA			((uint16_t) 0x0000);			/*!< PA[14] pin  */
#define AFIO_EXTICR4_EXTI14_PB			((uint16_t) 0x0100);			/*!< PB[14] pin  */
#define AFIO_EXTICR4_EXTI14_PC			((uint16_t) 0x0200);			/*!< PC[14] pin  */
#define AFIO_EXTICR4_EXTI14_PD			((uint16_t) 0x0300);			/*!< PD[14] pin  */
#define AFIO_EXTICR4_EXTI14_PE			((uint16_t) 0x0400);			/*!< PE[14] pin  */
#define AFIO_EXTICR4_EXTI14_PF			((uint16_t) 0x0500);			/*!< PF[14] pin  */
#define AFIO_EXTICR4_EXTI14_PG			((uint16_t) 0x0600);			/*!< PG[14] pin  */

/*!< EXTI3 Configuration  */
#define AFIO_EXTICR4_EXTI15_PA			((uint16_t) 0x0000);			/*!< PA[15] pin  */
#define AFIO_EXTICR4_EXTI15_PB			((uint16_t) 0x1000);			/*!< PB[15] pin  */
#define AFIO_EXTICR4_EXTI15_PC			((uint16_t) 0x2000);			/*!< PC[15] pin  */
#define AFIO_EXTICR4_EXTI15_PD			((uint16_t) 0x3000);			/*!< PD[15] pin  */
#define AFIO_EXTICR4_EXTI15_PE			((uint16_t) 0x4000);			/*!< PE[15] pin  */
#define AFIO_EXTICR4_EXTI15_PF			((uint16_t) 0x5000);			/*!< PF[15] pin  */
#define AFIO_EXTICR4_EXTI15_PG			((uint16_t) 0x6000);			/*!< PG[15] pin  */


/********************************************************************************************
* Bit position definitions of I2C peripherals
*********************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_SWRST					15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0					0
#define I2C_OAR1_ADD71					1
#define I2C_OAR1_ADD98					8
#define I2C_OAR1_ADDMODE				15

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RxNE					6
#define I2C_SR1_TxE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_SR2_CCR						0
#define I2C_SR2_DUTY					14
#define I2C_SR2_FS						15

//interrupts
#define EXTI 							((EXTI_RegDef_t*) EXTI_BASEADDR)
#define AFIO							((AFIO_RegDef_t*) AFIO_BASEADDR)
#endif /* STM32F103C6XX_H_ */
