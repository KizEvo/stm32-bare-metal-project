#include <stdint.h>

typedef struct {
	volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;

typedef struct {
	volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBSTR, CFGR2;
} RCC_TypeDef;

typedef struct {
	volatile uint32_t CTRL, LOAD, VAL, CALIB;
} STK_TypeDef;

typedef struct {
	volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

void delay(uint16_t ms);

// Refer to the STM32F103 datasheet
#define GPIO_Init(port)	((GPIO_TypeDef *)(0x40010800 + 0x400*(port)))
#define RCC_Init		((RCC_TypeDef *)(0x40021000))
#define USART1_Init		((USART_TypeDef *)(0x40013800))
// Refer to STM32F10xxx Cortex M3 programming manual
#define STK_Init    	((STK_TypeDef *)(0xE000E010))

enum GPIO_BANK {A, B, C, D, E, F, G};
enum PORT_CONFIG_CNF_INPUT 	{INPUT_ANALOG, INPUT_FLOATING, INPUT_PULL};
enum PORT_CONFIG_CNF_OUTPUT	{OUTPUT_PUSHPULL, OUTPUT_OPENDRAIN, OUTPUT_AF_PUSHPULL, OUTPUT_AF_OPENDRAIN};
enum PORT_CONFIG_MODE 		{MODE_INPUT, MODE_OUTPUT_MEDIUM, MODE_OUTPUT_LOW, MODE_OUTPUT_HIGH};
enum APB2_ENABLE_CLOCK		{AFIO_CK, IOPA_CK = 2, IOPB_CK, IOPC_CK, IOPD_CK, IOPE_CK, ADC1_CK = 9, ADC2_CK, TIM1_CK, SPI1_CK, USART1_CK = 14};

// Initialize peripherals
RCC_TypeDef *RCC = RCC_Init;				// Reset and clock control
GPIO_TypeDef *GPIOC = GPIO_Init(C);			// GPIOC
GPIO_TypeDef *GPIOA = GPIO_Init(A);			// GPIOA
STK_TypeDef *STK = STK_Init;				// SysTick
USART_TypeDef *USART1 = USART1_Init;		// USART1

void GPIO_PortConfig(GPIO_TypeDef *GPIO, uint8_t pin, uint8_t mode, uint8_t cnf)
{
	if(pin <= 7)
	{
		GPIO->CRL &= ~(0xF << (4*pin));
		GPIO->CRL |= (mode << (4*pin));
		GPIO->CRL |= ((cnf << 2) << (4*pin));
	}
	else
	{
		pin -= 8;
		GPIO->CRH &= ~(0xF << (4*pin));
		GPIO->CRH |= (mode << (4*pin));
		GPIO->CRH |= ((cnf << 2) << (4*pin));
	}
}

void RCC_SystemClockInit(void)
{
	// PLL Multiplication factor x4
	RCC->CFGR |= (0b0010 << 18);
	
	// Turn on HSEON flag
	RCC->CR |= (1 << 16);
	
	// Wait for HSERDY flag to be setted by hardware
	while(!((RCC->CR & 1 << 17) != 0));
	
	// Changing PLL entry clock source to HSE
	RCC->CFGR |= (1 << 16);
	
	// Turn off HSI clock
	RCC->CR |= (1 << 0);
	
	// Wait for PLLRDY flag to be cleared by hardware
	while((RCC->CR & 1 << 25) != 0);
	
	// Turn on PLL clock
	RCC->CR |= (1 << 24);
	
	// Pick PLL as main system clock source
	RCC->CFGR |= (0b10 << 0);
}


void USART1_TransmitConfig(void)
{
	// Word length = 8
	// 1 stop bit
	// Default
		
	// Enable USART transmitter
	USART1->CR1 |= (1 << 13);
	
	// Baud rate = f_CLK / (16*USARTDIV) = 32MHz / (16*208.333) = 9600 baud
	// USARTDIV = mantissa.(fraction / 16)
	uint32_t mantissa = 208;
	uint8_t fraction = 6;
	USART1->BRR = ((mantissa << 4) | fraction);

	// Start bit, (active low)
	USART1->CR1 |= (1 << 3);
}

void USART1_WriteBuffer(char byte)
{	
	// Write data to data register
	USART1->DR = byte;
	
	// Wait until transmit data register is empty
	while(!((USART1->SR & 1 << 7) != 0));
}

void USART_WriteString(char *string)
{
	for(uint16_t i = 0; string[i] != '\0'; i++)
		USART1_WriteBuffer(string[i]);
}

static inline void RCC_EnablePeripheralClock(uint8_t peripheral)
{
	RCC->APB2ENR &= ~(1 << peripheral);
	RCC->APB2ENR |= (1 << peripheral);
}

static inline void SysTick_Config(void)
{
	// Pick AHB/8 (= 4MHz) as clock source for SysTick
	STK->CTRL &= ~(1 << 2);
}

void delay(uint16_t ms)
{
	// Load RELOAD value of (3999 clock pulses)*ms before interrupt
	// 3999 clock pulses = 1ms
	STK->LOAD |= ((4000 - 1)*ms);
	
	// Clear counter value
	STK->VAL &= ~(0xFFFFFF);
	
	// Enable the counter
	STK->CTRL |= (1 << 0);
	
	// Wait for ms
	while(!((STK->CTRL & 1 << 16) != 0));
	
	// Disable the counter
	STK->CTRL &= ~(1 << 0);
}

int main(void)
{
	RCC_SystemClockInit();
	SysTick_Config();
	
	RCC_EnablePeripheralClock(AFIO_CK);
	RCC_EnablePeripheralClock(IOPC_CK);
	RCC_EnablePeripheralClock(IOPA_CK);
	RCC_EnablePeripheralClock(USART1_CK);
	
	// LED 13
	GPIO_PortConfig(GPIOC, 13, MODE_OUTPUT_LOW, OUTPUT_PUSHPULL);
	// UART Tx, Rx 
	GPIO_PortConfig(GPIOA, 9, MODE_OUTPUT_HIGH, OUTPUT_AF_PUSHPULL);
	GPIO_PortConfig(GPIOA, 10, MODE_INPUT, INPUT_FLOATING);
	
	USART1_TransmitConfig();
	
	while(1)
	{
		GPIOC->ODR &= ~(1 << 13);
		USART_WriteString("Hello World\r\n");
		delay(500);
		GPIOC->ODR |= (1 << 13);
		delay(500);
	}
	return 0;
}

// Startup code
__attribute__((noreturn, naked)) void Reset_Handler(void)
{
	// Start and end address of different sections defined in linker.ld
	extern uint32_t _sdata, _edata, _sbss, _ebss, _sizedata;
	
	// .bss section, zero initialize variables
	for(uint32_t *dest = &_sbss; dest < &_ebss; dest++)
	{
		*dest = 0;
	}
	
	// .data section, copy from flash to sram
	uint32_t *src = &_sizedata, *dest = &_sdata;
	while(dest < &_edata)
	{
		*dest++ = *src++;
	}
	
	// Call main()
	main();
	
	// Loop forever if main() return
	while (1);
}

// Stack pointer, defined in linker.ld
extern void _estack(void);

// Vector Handlers: Systems
__attribute__ ((weak, alias ("Default_Handler"))) void NMI_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void HardFault_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void MemManage_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void BusFault_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void UsageFault_Handler(void);
	// Reserved 0x0000_001C - 0x0000_002B (0 0 0 0)
__attribute__ ((weak, alias ("Default_Handler"))) void SVCall_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DebugMonitor_Handler(void);
	// Reserved 0x0000_0034
__attribute__ ((weak, alias ("Default_Handler"))) void PendSV_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void SysTick_Handler(void);
// Vector Handlers: Interrupts
__attribute__ ((weak, alias ("Default_Handler"))) void WWDG_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void PVD_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TAMPER_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void RTC_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void FLASH_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void RCC_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI0_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI4_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel4_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel5_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel6_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA1_Channel7_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void ADC1_2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USB_HP_CAN__Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USB_LP_CAN__Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void CAN_RX1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void CAN_SCE_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI9_5_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM1_BRK_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM1_UP_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM1_TRG_COM_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM1_CC_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM4_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void I2C1_EV_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void I2C1_ER_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void I2C2_EV_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void I2C2_ER_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void SPI1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void SPI2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USART1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USART2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USART3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void EXTI15_10_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void RTCAlarm_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void USBWakeup_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM8_BRK_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM8_UP_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM8_TRG_COM_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM8_CC_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void ADC3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void FSMC_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void SDIO_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM5_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void SPI3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void UART4_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void UART5_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM6_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void TIM7_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA2_Channel1_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA2_Channel2_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA2_Channel3_Handler(void);
__attribute__ ((weak, alias ("Default_Handler"))) void DMA2_Channel4_5_Handler(void);

// Vector tables
__attribute__((section(".vectors"))) void (*const tables[])(void) = {
	_estack,
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVCall_Handler,
	DebugMonitor_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,
	WWDG_Handler,
	PVD_Handler,
	TAMPER_Handler,
	RTC_Handler,
	FLASH_Handler,
	RCC_Handler,
	EXTI0_Handler,
	EXTI1_Handler,
	EXTI2_Handler,
	EXTI3_Handler,
	EXTI4_Handler,
	DMA1_Channel1_Handler,
	DMA1_Channel2_Handler,
	DMA1_Channel3_Handler,
	DMA1_Channel4_Handler,
	DMA1_Channel5_Handler,
	DMA1_Channel6_Handler,
	DMA1_Channel7_Handler,
	ADC1_2_Handler,
	USB_HP_CAN__Handler,
	USB_LP_CAN__Handler,
	CAN_RX1_Handler,
	CAN_SCE_Handler,
	EXTI9_5_Handler,
	TIM1_BRK_Handler,
	TIM1_UP_Handler,
	TIM1_TRG_COM_Handler,
	TIM1_CC_Handler,
	TIM2_Handler,
	TIM3_Handler,
	TIM4_Handler,
	I2C1_EV_Handler,
	I2C1_ER_Handler,
	I2C2_EV_Handler,
	I2C2_ER_Handler,
	SPI1_Handler,
	SPI2_Handler,
	USART1_Handler,
	USART2_Handler,
	USART3_Handler,
	EXTI15_10_Handler,
	RTCAlarm_Handler,
	USBWakeup_Handler,
	TIM8_BRK_Handler,
	TIM8_UP_Handler,
	TIM8_TRG_COM_Handler,
	TIM8_CC_Handler,
	ADC3_Handler,
	FSMC_Handler,
	SDIO_Handler,
	TIM5_Handler,
	SPI3_Handler,
	UART4_Handler,
	UART5_Handler,
	TIM6_Handler,
	TIM7_Handler,
	DMA2_Channel1_Handler,
	DMA2_Channel2_Handler,
	DMA2_Channel3_Handler,
	DMA2_Channel4_5_Handler
};

void Default_Handler(void)
{
	while(1);
}