#include <stdint.h>

extern int main(void);

typedef struct {
	volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;

typedef struct {
	volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR, AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBSTR, CFGR2;
} RCC_TypeDef;

typedef struct {
	volatile uint32_t ISER[3], ICER[3], ISPR[3], ICPR[3], IABR[3], IPR[21];
} NVIC_TypeDef;

typedef struct {
	volatile uint32_t CTRL, LOAD, VAL, CALIB;
} STK_TypeDef;

typedef struct {
	volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

typedef struct {
	volatile uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR1, JOFR2, JOFR3, JOFR4, HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR1, JDR2, JDR3, JDR4, DR;
} ADC_TypeDef;

void delay(uint16_t ms);

// Refer to the STM32F103 datasheet
#define GPIO_Init(port)	((GPIO_TypeDef *)(0x40010800 + 0x400*(port)))
#define RCC_Init		((RCC_TypeDef *)(0x40021000))
#define USART1_Init		((USART_TypeDef *)(0x40013800))
#define ADC1_Init		((ADC_TypeDef *)(0x40012400))
// Refer to STM32F10xxx Cortex M3 programming manual
#define NVIC_Init		((NVIC_TypeDef *)(0xE000E100))
#define STK_Init    	((STK_TypeDef *)(0xE000E010))

enum GPIO_BANK {A, B, C, D, E, F, G};
enum PORT_CONFIG_CNF_INPUT 	{INPUT_ANALOG, INPUT_FLOATING, INPUT_PULL};
enum PORT_CONFIG_CNF_OUTPUT	{OUTPUT_PUSHPULL, OUTPUT_OPENDRAIN, OUTPUT_AF_PUSHPULL, OUTPUT_AF_OPENDRAIN};
enum PORT_CONFIG_MODE 		{MODE_INPUT, MODE_OUTPUT_MEDIUM, MODE_OUTPUT_LOW, MODE_OUTPUT_HIGH};
enum APB2_ENABLE_CLOCK		{AFIO_CK, IOPA_CK = 2, IOPB_CK, IOPC_CK, IOPD_CK, IOPE_CK, ADC1_CK = 9, ADC2_CK, TIM1_CK, SPI1_CK, USART1_CK = 14};

// Initialize peripherals
RCC_TypeDef *RCC = RCC_Init;				// Reset and clock control
NVIC_TypeDef *NVIC = NVIC_Init;				// Nested vector interrupt controller NVIC
GPIO_TypeDef *GPIOC = GPIO_Init(C);			// GPIOC
GPIO_TypeDef *GPIOA = GPIO_Init(A);			// GPIOA
STK_TypeDef *STK = STK_Init;				// SysTick
USART_TypeDef *USART1 = USART1_Init;		// USART1
ADC_TypeDef *ADC1 = ADC1_Init;				// ADC1

// Global variables
volatile uint16_t ADC_CAL_ERR_VAL;			// ADC calibration value
volatile uint8_t UART_RX_VAL;					// Value read from UART receive interrupt

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

void NVIC_EnableIRQ(uint32_t position)
{
	// Read previous enabled IRQ
	uint32_t prev_enabled_irq = NVIC->ISER[position >> 5];
	// Write the previous enabled IRQ with the new IRQ position
	NVIC->ISER[position >> 5] = prev_enabled_irq | (1 << (position - 32*(position >> 5)));
}

void USART1_TransmitReceiveConfig(void)
{
	// Word length = 8
	// 1 stop bit
	// Default
		
	// Enable USART1
	USART1->CR1 |= (1 << 13);
	
	// Baud rate = f_CLK / (16*USARTDIV) = 32MHz / (16*208.375) = 9598.08 baud
	// USARTDIV = mantissa.(fraction / 16)
	uint32_t mantissa = 208;
	uint8_t fraction = 6;
	USART1->BRR = ((mantissa << 4) | fraction);

	// Start bit, (active low) - transmission
	USART1->CR1 |= (1 << 3);
	
	// Enable receive interrupt, RXNEIE bit
	USART1->CR1 |= (1 << 5);
	
	// Enable NVIC for UART
	NVIC_EnableIRQ(37);
	
	// Set RE bit (look for start bit) - receive
	USART1->CR1 |= (1 << 2);
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

void ADC1_Config(void)
{
	// Sample time = 1.5 clock cycles
	// Data right alignment in ADC_DR
	// Default
	
	// Continuous conversion
	ADC1->CR2 |= (1 << 1);
	
	// Perform 1 conversion in the regular channel conversion sequence
	ADC1->SQR1 &= ~(0xF << 20);
	
	// Set ADC1 channel 1 is the first sequence to be converted
	// Since we tell it to do 1 conversion only in the sequence 
	// so this is the only channel of the ADC to be converted
	ADC1->SQR3 |= (1 << 0);
	
	// Enable external trigger conversion mode
	ADC1->CR2 |= (1 << 20);
	
	// Select external trigger event for regular channel, 0b111 = SWSTART
	ADC1->CR2 |= (0b111 << 17);
	
	// Turn on ADC
	ADC1->CR2 |= (1 << 0);
	
	// Wait for few clocks before starting calibration
	delay(12);
	
	// Start ADC calibration
	ADC1->CR2 |= (1 << 2);
	
	// Wait until calibration is finished (hardware clears this bit)
	while((ADC1->CR2 & 1 << 2) != 0);
	
	// Read the calibration error value in ADC_DR
	ADC_CAL_ERR_VAL = ADC1->DR;
	
	// Wait for few clocks t_STAB
	delay(48);
}

char *numbToString(uint32_t numb)
{
    static char temp[32], string[33];
    uint8_t idx = 0;
    while(numb != 0)
    {
        temp[idx++] = (numb % 10) + '0';
        numb = numb / 10;
    }
	// Reverse the string order
    for(uint8_t i = 0; i < idx; i++)
    {
        string[i] = temp[idx - i - 1];
    }
	// Null terminated char
    string[idx] = '\0';
    return string;
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
	RCC_EnablePeripheralClock(ADC1_CK);
	
	// LED 13
	GPIO_PortConfig(GPIOC, 13, MODE_OUTPUT_LOW, OUTPUT_PUSHPULL);
	// UART Tx, Rx - refers to GPIO configurations for device peripherals /pg.166
	GPIO_PortConfig(GPIOA, 9, MODE_OUTPUT_HIGH, OUTPUT_AF_PUSHPULL);
	GPIO_PortConfig(GPIOA, 10, MODE_INPUT, INPUT_FLOATING);
	// ADC1 channel 1
	GPIO_PortConfig(GPIOA, 1, MODE_INPUT, INPUT_ANALOG);
	
	USART1_TransmitReceiveConfig();
	ADC1_Config();
	
	// start ADC conversion of regular channel
	ADC1->CR2 |= (1 << 22); 
	
	while(1)
	{
		if((ADC1->SR & 1 << 1) != 0 && UART_RX_VAL == '1')
		{
			uint16_t adc_raw_data = ADC1->DR;
			adc_raw_data -= ADC_CAL_ERR_VAL;
			// Fixed point arithmetic, precision 1e-1
			uint32_t voltage_divider = ((adc_raw_data * 10) * (33))/40960;
			uint32_t pot_val = 20000/(((330) / voltage_divider) - 10);
			char *pot_val_str = numbToString(pot_val);
			USART_WriteString(pot_val_str);
			USART_WriteString("\r\n");
			ADC1->SR &= ~(1 << 1);	// clears ADC EOC flag
			ADC1->CR2 |= (1 << 22); // start ADC conversion of regular channel
		} else
		{
			USART_WriteString("No action\r\n");
		}
		GPIOC->ODR &= ~(1 << 13);
		delay(500);
		GPIOC->ODR |= (1 << 13);
		delay(500);
	}
	return 0;
}

void USART1_Handler(void)
{
	// Read received data, only need 0-6 data bit incase there is parity bit at the MSB
	UART_RX_VAL = USART1->DR & 0x7F;
	// Clear rx flag
	USART1->SR &= ~(1 << 5);
}