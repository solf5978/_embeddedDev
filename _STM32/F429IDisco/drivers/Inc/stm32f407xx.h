#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * BASE ADDR. OF FLASH / SRAM MEMORIES
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x20001C00U
#define ROM	BASEADDR				0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR


#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/*
 * BASE ADDR OF PHERIPHERALS ON AHB1 BUS
 */
#define GPIO_A_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIO_B_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIO_C_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIO_D_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIO_E_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIO_F_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIO_G_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIO_H_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIO_I_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000U)

/*
 * BASE ADDR OF PHERIPHERALS ON APB1 BUS
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00U)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800U)

#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000U)

/*
 * BASE ADDR OF PHERIPHERALS ON APB2 BUS
 */
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00U)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000U)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800U)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400U)


/*
 **************************************************
 * REGISTERS OF A PERIPHERAL TO MCU				  *
 * STM32F4X FAMILY								  *
 **************************************************
 */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_RegDef_t;

typedef struct
{
  volatile uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  volatile uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  volatile uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t 			RESERVED0;     /*!< Reserved, 0x1C                                                       */
  volatile uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      	RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  volatile uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      	RESERVED2;     /*!< Reserved, 0x3C                                                       */
  volatile uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      	RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  volatile uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      	RESERVED4;     /*!< Reserved, 0x5C                                                       */
  volatile uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      	RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  volatile uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  volatile uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      	RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  volatile uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  volatile uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  volatile uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  volatile uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  volatile uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;


/*
 * PERIPHERAL DEFINITIONS
 */

#define GPIOA						((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG						((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOI						((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC							((RCC_RegDef_t *)RCC_BASEADDR)

#endif /* INC_STM32F407XX_H_ */
