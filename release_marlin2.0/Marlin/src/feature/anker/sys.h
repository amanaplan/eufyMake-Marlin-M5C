#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 
 
// Bit-band operations, implementing GPIO control functions similar to the 51.
// For detailed implementation ideas, refer to Chapter 5 (pages 87-92) of the
//   <<CM3 Definitive Guide>>. The M4 is similar to the M3, except that the
//   register addresses are different.
// IO Port Operation Macro Definitions
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
// IO Port Address Mapping
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 

// IO Port Operation, only for a single IO port
// Make sure the value of n is less than 16
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // Output
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // Input

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  // Output
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  // Input 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  // Output
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  // Input

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  // Output
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  // Input

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  // Output
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  // Input

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  // Output
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  // Input

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  // Output 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  // Input

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  // Output 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  // Input

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  // Output 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  // Input
////////////////////////////////////////////////////////////////////////////////// 
//Ex_NVIC_Config Dedicated Definition
#define GPIO_A 				0
#define GPIO_B 				1
#define GPIO_C				2
#define GPIO_D 				3
#define GPIO_E 				4
#define GPIO_F 				5
#define GPIO_G 				6 
#define GPIO_H 				7 
#define GPIO_I 				8 

#define FTIR   				1  		// Trigger on Falling Edge
#define RTIR   				2  		// Trigger on Rising Edge

// GPIO Setting Dedicated Macro Definition
#define GPIO_MODE_IN    	0		// Normal Input Mode
#define GPIO_MODE_OUT		1		// Normal Output Mode
#define GPIO_MODE_AF		2		// Alternate Function Mode
#define GPIO_MODE_AIN		3		// Analog Input Mode

#define GPIO_SPEED_2M		0		// GPIO Speed 2Mhz
#define GPIO_SPEED_25M		1		// GPIO Speed 25Mhz
#define GPIO_SPEED_50M		2		// GPIO Speed 50Mhz
#define GPIO_SPEED_100M		3		// GPIO Speed 100Mhz

#define GPIO_PUPD_NONE		0		// No Pull-Up or Pull-Down
#define GPIO_PUPD_PU		1		// Pull-Up
#define GPIO_PUPD_PD		2		// Pull-Down
#define GPIO_PUPD_RES		3		// Reserved 

#define GPIO_OTYPE_PP		0		// Push-Pull Output
#define GPIO_OTYPE_OD		1		// Open-Drain Output

// GPIO PIN Number Definition
#define PIN0				1<<0
#define PIN1				1<<1
#define PIN2				1<<2
#define PIN3				1<<3
#define PIN4				1<<4
#define PIN5				1<<5
#define PIN6				1<<6
#define PIN7				1<<7
#define PIN8				1<<8
#define PIN9				1<<9
#define PIN10				1<<10
#define PIN11				1<<11
#define PIN12				1<<12
#define PIN13				1<<13
#define PIN14				1<<14
#define PIN15				1<<15 

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
////////////////////////////////////////////////////////////////////////////////// 
u8 Sys_Clock_Set(u32 plln,u32 pllm,u32 pllp,u32 pllq);		// System Clock Settings
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq); // Clock Initialization 
void Sys_Soft_Reset(void);      							// System Soft Reset
void Sys_Standby(void);         							// System Standby Mode	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);	// Set Offset Address
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);			// Set NVIC Group
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);// Setup interrupts
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);				// External interrupt configuration function (GPIOA to I only)
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);		// GPIO Multiplexing Function Settings
void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD); // GPIO Setup Function
// The following is the assembly code
void WFI_SET(void);		// Execute WFI Instruction
void INTX_DISABLE(void); // Disable All Interrupts
void INTX_ENABLE(void);	// Enable All Interrupts
#endif
