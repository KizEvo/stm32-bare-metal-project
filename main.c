#include <stdint.h>

typedef struct {
	volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} _GPIO;

typedef struct {
	volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, 
						AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBSTR, CFGR2;
} _RCC;

enum GPIO_BANK {A, B, C, D, E, F, G};

#define GPIO_Init(port)	((_GPIO *)(0x40010800 + 0x400*(port)))
#define RCC_Init		((_RCC *)(0x40021000))

enum PORT_CONFIG_CNF_INPUT 	{INPUT_ANALOG, INPUT_FLOATING, INPUT_PULL};
enum PORT_CONFIG_CNF_OUTPUT	{OUTPUT_PUSHPULL, OUTPUT_OPENDRAIN, OUTPUT_AF_PUSHPULL, OUTPUT_AF_OPENDRAIN};
enum PORT_CONFIG_MODE 		{MODE_INPUT, MODE_OUTPUT_MEDIUM, MODE_OUTPUT_LOW, MODE_OUTPUT_HIGH};
enum APB2_ENABLE_CLOCK		{AFIO, IOPA = 2, IOPB, IOPC, IOPD, IOPE, ADC1 = 9, ADC2, TIM1, SPI1, USART1 = 14};

// Initialize peripherals
_RCC *RCC = RCC_Init;			// Reset and clock control
_GPIO *GPIOC = GPIO_Init(C);	// GPIOC

void GPIO_PortConfig(_GPIO *GPIO, uint8_t pin, uint8_t mode, uint8_t cnf)
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
	// Wait for HSIRDY flag to be enabled by hardware
	while(!((RCC->CR & 1 << 1) != 0));
	// Pick HSI as main system clock source
	RCC->CFGR &= ~(3 << 0);
}

void RCC_EnablePeripheralClock(uint8_t peripheral)
{
	RCC->APB2ENR &= ~(1 << peripheral);
	RCC->APB2ENR |= (1 << peripheral);
}

void delay(uint32_t counter)
{
	uint32_t temp = counter;
	while(counter > 0)
	{
		counter--;
		while(temp > 0)
		{
			temp--;
		}			
	}
}

int main(void)
{
	RCC_SystemClockInit();
	RCC_EnablePeripheralClock(IOPC);
	GPIO_PortConfig(GPIOC, 13, MODE_OUTPUT_LOW, OUTPUT_PUSHPULL);
	
	while(1)
	{
		GPIOC->ODR &= ~(1 << 13);
		delay(10000);
		GPIOC->ODR |= (1 << 13);
		delay(10000);
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