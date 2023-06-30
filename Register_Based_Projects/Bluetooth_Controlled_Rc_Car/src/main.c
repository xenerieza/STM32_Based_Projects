#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define IN1	0x02
#define IN2 0x03
#define IN3 0x05
#define IN4 0x04

#define Forward_Command	  'F'
#define Backward_Command  'B'
#define	Left_Command 	  'L'
#define Right_Command	  'R'
#define	Stop_Command	  'S'

void RCC_Config(void)
{
	/*RCC Configuration Steps
	 * 1- Reset Control Register.
	 * 2- High Speed Internal Disabled
	 * 3- High Speed External Enabled and Wait HSE Flag Ready.
	 * 4- Set the Clock Security Enabled to 1 (CSS -> ON).
	 * 5- Set the Power Enable Clock and Voltage Regulator.
	 * 6- Configurate the Flash Prefecth and Latency.
	 * 7- Configurate the PLL Settings (M=4, N=168, P=2, SRC=HSE Enabled, AHB=1, APB1=4, APB2=2 after all HCLK: 168MHz)
	 * 8- Enable PLL and wait PLL Flag Ready.
	 * 9- Select the Clock Source and wait for it to be set.
	 */

	// Reset Control Registers.
	RCC -> CR &= 0x00000083;

	// High Speed Internal Disabled.
	RCC -> CR &= ~(1 << 0);

	// High Speed External Enabled.
	RCC -> CR |= 0x00010000;		//(1 << 16)

	// Wait the HSE Flag Ready.
	while(!(RCC -> CR & (0x00020000)));

	// Enabled the CSS
	RCC -> CR |= 0x00080000; // (1 << 19) CSS Enabled.

	// Set Power Enable Clock On. (Page: 246 APB1ENR Register PWREN Bits.)
	RCC -> APB1ENR |= 0x10000001;			//(1 << 28)
	//Set Power Voltage Regulator Scale 1. (PWR_CR[14:15] --> 11) (Page: 145 PWR Control Register Bits VOS)
	PWR -> CR |= 0x0000C000;		//Bit 14 and 15 set to 1 	//PWR -> CR |= (1 << 14), PWR -> CR |= (1 << 15);


	// Configurate Flash Prefecth and Latency. Flash Access Control Register (ACR) (Page: 98 ACR)
	FLASH -> ACR |= 0x00000200;	// Instruction Cache Enabled.
	FLASH -> ACR |= 0x00000100;	// Prefecth Buffer Enabled.
	FLASH -> ACR |= 0x00000400;	// Data Cache Enabled.
	FLASH -> ACR |= 0x00000005; // Flash Latency Wait State set 5.

	// Configurate PLL Settings (M=4, N=168, P=2, AHB=1, APB1=4, APB2=2 after all HCLK: 168MHz)
	// SET PLL Configuration.
	// HSE Oscillator selected as PLL and PLLI2S clock entry. (PLL Source HSE)
	RCC -> PLLCFGR |= 0x00400000;
	//PLLM and PLLN Configuration.
	RCC -> PLLCFGR |= 0x00000004;	// ~(1 << 0) | ~(1 << 1) | (1 << 2) | ~(1 << 3) | ~(1 << 5) btw. PLLM = 4.
	RCC -> PLLCFGR |= 0x000000A8;	// PLL N= 168 --> 3, 5 and 7 set to 1.

	// P=2 CFGR[17:16] to 00.	RCC -> PLLCFGR |= 0x00000000;
	RCC -> PLLCFGR &= ~(1 << 16);
	RCC -> PLLCFGR &= ~(1 << 17);


	// AHB1 Prescaler Div1.
	RCC -> CFGR |= 0x00000000;
	// APB1 Prescaler Div4.
	RCC -> CFGR |= 0x00001400;
	// APB2 Prescaler Div2.
	RCC -> CFGR |= 0x00008000;


	// Enable PLL and wait PLL Flag Ready.
	// PLL Enabled.
	RCC -> CR |= 0x01000000;
	// Wait PLL Flag Ready.
	while(!(RCC -> CR & (0x02000000)));

	// Select the Clock Source and wait for it to be set.
	// PLL selected as system clock source.
	RCC -> CFGR |= 0x00000002;
	// Wait System Clock Switch Status to set.
	while(!(RCC -> CFGR & (0x00000008)));

	/*Test System Coreclock in main C
	 * Call your RCC_Config() in main
	 * Create uint32_t systemclock on global.
	 * extern uint32_t SystemCoreClock
	 * RCC_Config();
	 * SystemCoreClockUpdate();
	 * systemclock = SystemCoreClock;
	 * Set breakpoint and observe your systemclock.
	*/
}

void GPIO_Config(void)
{
	// Configurate GPIO Pins for Motor Drivers. (Output)
	// GPIOE RCC Clock Enable.
	RCC -> AHB1ENR   |= RCC_AHB1ENR_GPIOEEN;  //0x00000010;
	// Pin 2,3,4,5 --> Output. GPIO_Output: 01 Moder[11:4] --> 01010101
	GPIOE -> MODER   |= 0x00000550;	 // ~(1 << 11) | (1 << 10) | ~(1 << 9) | (1 << 8) | ~(1 << 7) | (1 << 6) | ~(1 << 5) | (1 << 4)
	GPIOE -> OTYPER  |= 0x00000000;  // Set to output push pull.
	GPIOE -> OSPEEDR |= 0x00000000;	 // Set to low frequency.
	GPIOE -> PUPDR   |= 0x00000000;	 // 2,3,4,5 no pull
}

void USART_Config(void)
{
	// Enable RCC Clock APB1 for USART2
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	// Enable GPIO Clock AHB1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// Set Pin2: Tx AF, Pin3: Rx AF.
	GPIOA -> MODER |= GPIO_MODER_MODER2_1;	// 0x00000020
	GPIOA -> MODER |= GPIO_MODER_MODER3_1;	// 0x00000080
	GPIOA -> AFR[0] |= (7 << 8);	// Pa2: Tx set alternate functions.
	GPIOA -> AFR[0] |= (7 << 12);	// Pa3: Rx set alternate functions.

	/*	Baudrate Calculation
		 *	Baudrate = (fclck(42MHz(APB1 Peripheral Clock))) / (16 x USARTDIV)
		 *	USARTDIV = (fclck(42MHz)) / (16 x Baudrate(38400))
		 *	USARTDIV = 68.3593
		 *	Mantissa = 68
		 *	Fraction = 16 * 0.3593 = 5,7488
		 *	Fraction = 6 | 6
		 *	USART2 -> BRR = (6(Fraction) << 0) | (68(Mantissa) << 4) = (68 << 4 = 0x44= + (6 << 0 = 6) = 0x446
		 */
	USART2 -> BRR = 0x446;

	// Enable the Receiver(Rx) Set to 1
	USART2 -> CR1 |= USART_CR1_RE;	//0x00000004 or (1 << 2)

	// Not required for this project. Enable the Transmitter. Set to 1
	USART2 -> CR1 |= USART_CR1_TE;	//0x00000008 or (1 << 3)

	// Parity Control Disabled. (Parity None)
	USART2 -> CR1 &= ~(1 << 10);

	// Wordlength 8bit (0 << 12 )
	USART2 -> CR1 &= ~(1 << 12);

	// Oversampling mode 16 --> CR1[15] set 0
	USART2 -> CR1 &= ~(1 << 15);

	// CR[13:12] Set 00 to Stopbit1
	USART2 -> CR2 &= ~(1 << 13);
	USART2 -> CR2 &= ~(1 << 12);	 // 0x00000000

	// Enable USART2 CR1[13] -> 1 -> (1 << 13).
	USART2 -> CR1 |= USART_CR1_UE;	 // 0x00002000 or (1 << 13)
}

void GPIO_MotorSetBit(uint16_t pinNum)
{
	GPIOE -> ODR |= (1 << pinNum);
}

void GPIO_MotorResetBit(uint16_t pinNum)
{
	GPIOE -> ODR &= ~(1 << pinNum);
}


/*
void GPIO_SetBit(GPIO_InitTypeDef* GPIOx ,uint16_t pinNum)
{
	// Set the specified GPIO Port and Pin number.
	GPIOx -> ODR |= (1 << pinNum);
}

void GPIO_ResetBit(GPIO_InitTypeDef* GPIOx ,uint16_t pinNum)
{
	// Reset the specified GPIO Port and Pin number.
	GPIOx -> ODR &= ~(1 << pinNum);
}*/

void Forward(void)
{
	//1010
	GPIO_MotorSetBit(IN1);
	GPIO_MotorResetBit(IN2);
	GPIO_MotorSetBit(IN3);
	GPIO_MotorResetBit(IN4);
}

void Backward(void)
{
	//0101
	GPIO_MotorResetBit(IN1);
	GPIO_MotorSetBit(IN2);
	GPIO_MotorResetBit(IN3);
	GPIO_MotorSetBit(IN4);
}

void Turn_Right(void)
{
	//1001
	GPIO_MotorSetBit(IN1);
	GPIO_MotorResetBit(IN2);
	GPIO_MotorResetBit(IN3);
	GPIO_MotorSetBit(IN4);
}

void Turn_Left(void)
{
	//0110
	GPIO_MotorResetBit(IN1);
	GPIO_MotorSetBit(IN2);
	GPIO_MotorSetBit(IN3);
	GPIO_MotorResetBit(IN4);
}

void Stop(void)
{
	GPIO_MotorResetBit(IN1);
	GPIO_MotorResetBit(IN2);
	GPIO_MotorResetBit(IN3);
	GPIO_MotorResetBit(IN4);
}

void GetCommand(uint8_t command)
{
	switch(command)
	{
	case 'F':
		Forward();
		break;
	case 'B':
		Backward();
		break;
	case 'R':
		Turn_Right();
		break;
	case 'L':
		Turn_Left();
		break;
	case 'S':
		Stop();
		break;
	default:
		Stop();
		break;
	}
}


void delay(uint32_t time)
{

	// It takes 8 cycles until this while loop completes.
	// When you divide your clock speed by 8, the remaining value corresponds to 1 seconds.
	// My clock speed is 168MHz. 168/8 = 21
	// The value 21000000 corresponds to 1 second.

	while(time--);
}

int main(void)
{
	RCC_Config();
	GPIO_Config();
	USART_Config();
	while(1)
	{
		while(!(USART2 -> SR & 0x00000020));
		uint8_t command = USART2 -> DR;
		GetCommand(command);

	}
}




void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
  /* TODO, implement your code here */
  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  /* TODO, implement your code here */
  return -1;
}
