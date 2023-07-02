#include "STM32F10x.h"
#include <cmsis_os.h>

//Pin definition
//LEDs
#define LED1 0  //PA0
#define LED2 1  //PA1
#define LED3 2  //PA2
#define LED4 15 //PA15
#define LED5 8  //PA8
#define LED6 6  //PA6
#define LED7 5  //PA5
#define LED8 11 //PA11

//LCD display
#define LCD_RS 15
#define LCD_EN 12

//Switches
#define SW1 12  //PB12
#define SW2 13  //PB13
#define SW3 14  //PB14
#define SW4 15  //PB15
#define SW5 5   //PB5
#define SW6 4   //PB4
#define SW7 3   //PB3
#define SW8 3   //PA3
#define SW9 4   //PA4
#define SW10 8  //PB8
#define SW11 9  //PB9
#define SW12 11 //PB11
#define SW13 10 //PB10
#define SW14 7  //PA7
#define SW15 15 //PC15
#define SW16 14 //PC14
#define SW17 13 //PC13

//Potentiometer
#define POT 1 //PB1

//Buzzer
#define BUZ 0 //PB0

int val01_delay = 1000000;
int val02_delay = 1000000;

/*----------------------------------------------------------------------------
  Simple delay routine
 *---------------------------------------------------------------------------*/
void delay (unsigned int count)
{
	unsigned int index;

	for(index =0; index < count; index++)
	{
		;
	}
}



/*----------------------------------------------------------------------------
  RedPill setup routine
 *---------------------------------------------------------------------------*/
void setup_RedPill()
{
	// int16_t swa, swb, swc;  //Variables to read the switches according to the port it is connected
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AF clock
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	delay(100);

	RCC->APB2ENR |= 0xFC | (1 << 9);   // ENABLE clocks for GPIOs and ADC1
	// Setting up outputs for leds
	ADC1->CR2 = 1; /* ADON = 1 (power-up) */
	ADC1->CR2 |= (1 << 2); // enable calibration
	ADC1->SMPR2 = 1 << 3; /* SMP1 = 001 */
	delay(1); /* wait 1us to make sure the adc module is stable */
	GPIOA->CRL = 0x43344333;    // PA3, PA4 and PA7: inputs (switches)
	GPIOA->CRH = 0x33333333;    // PA8 - PA15: outputs (leds)

	// Settig up inputs for switches
	GPIOB->CRL = 0x4444440B; // PB0 set for output+alternate wave form, since it is connected to buzzer.
	GPIOB->CRH = 0x44444444;
	GPIOB->ODR = 0xF000; // set pull-up in PB12 - PB15
	GPIOC->CRH = 0x44444444;
	GPIOC->ODR = 0xFFFFFFFF; // set pull-up in GPIOC

	delay(1); // wait for I/O setup
	GPIOA->ODR &= ~(1 << LCD_RS); // Turn off LED4
	delay(1); // wait for LED4 to turn off
}

/*----------------------------------------------------------------------------
  LED setup routines
 *---------------------------------------------------------------------------*/
// Test for LEDs
void ledsON()
{
	GPIOA->ODR |= (1 << LED1);    // led 1 - PA0
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED2);    // led 2 - PA1
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED3);    // led 3 - PA2
	delay(val01_delay);
	GPIOA->ODR |= (1 << LCD_RS);  // led 4 - PA15
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED5);    // led 5 - PA8
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED6);    // led 6 - PA6
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED7);    // led 7 - PA5
	delay(val01_delay);
	GPIOA->ODR |= (1 << LED8);    // led 8 - PA11
	delay(val01_delay);
}

void ledsOFF()
{
	GPIOA->ODR &= ~(1 << LED1);    // led 1
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED2);    // led 3
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED3);    // led 3
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LCD_RS);  // led 4
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED5);    // led 5
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED6);    // led 6
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED7);    // led 7
	delay(val02_delay);
	GPIOA->ODR &= ~(1 << LED8);    // led 8
	delay(val02_delay);
}

/*----------------------------------------------------------------------------
  Flash LED 1
 *---------------------------------------------------------------------------*/
void led_thread1(void const *argument)
{
	for (;;)
	{
		GPIOA->ODR |= (1 << LED1);    // led 1 - PA0
		delay(val01_delay);
		GPIOA->ODR &= ~(1 << LED1);    // led 1 - PA0
		delay(val01_delay);
	}
}

/*----------------------------------------------------------------------------
  Flash LED 2
 *---------------------------------------------------------------------------*/
void led_thread2(void const *argument)
{
	for (;;)
	{
		GPIOA->ODR |= (1 << LED2);    // led 2 - PA2
		delay(val02_delay);
		GPIOA->ODR &= ~(1 << LED2);    // led 2 - PA2
		delay(val02_delay);
	}
}

void func3()
{
	GPIOA->ODR ^= (1 << LED7);    // Toggle led 7 - PA5
	delay(val01_delay);
	GPIOA->ODR ^= (1 << LED7);    // Toggle led 7 - PA5
	delay(val01_delay);
}

void func2()
{
	GPIOA->ODR ^= (1 << LED3);    // Toggle led 3 - PA2
	delay(val01_delay);
	GPIOA->ODR ^= (1 << LED3);    // Toggle led 3 - PA2
	delay(val01_delay);
}

void func1()
{
	GPIOA->ODR ^= (1 << LED5);    // Toggle led 5 - PA8
	delay(val01_delay);
	GPIOA->ODR ^= (1 << LED5);    // Toggle led 5 - PA8
	delay(val01_delay);
}

void select()
{
	if ((GPIOB->IDR & (1 << SW1)) == 0)
	{ // reads switch 1
		func1();
	}
	if ((GPIOB->IDR & (1 << SW2)) == 0)
	{ // reads switch 2
		func2();
	}
	if ((GPIOB->IDR & (1 << SW3)) == 0)
	{ // reads switch 3
		func3();
	}
}

/*----------------------------------------------------------------------------
 Define the thread handles and thread parameters
 *---------------------------------------------------------------------------*/

osThreadId main_ID, led_ID1, led_ID2, led_func1, led_func2, led_func3; // create IDS
osThreadDef(led_thread1, osPriorityNormal, 1, 0);
osThreadDef(led_thread2, osPriorityNormal, 1, 0);
osThreadDef(func1, osPriorityNormal, 1, 0); // creates a thread
osThreadDef(func2, osPriorityNormal, 1, 0); // creates a thread
osThreadDef(func3, osPriorityNormal, 1, 0); // creates a thread

int main(void)
{
	osKernelInitialize();                    // initialize CMSIS-RTOS

	setup_RedPill(); // prepares board

	led_func1 = osThreadCreate(osThread(func1), NULL); // saves thread for leds 1
	led_func2 = osThreadCreate(osThread(func2), NULL); // saves thread for leds 2
	led_func3 = osThreadCreate(osThread(func3), NULL); // saves thread for leds 3

	osKernelStart();                         // start thread execution
	for (;;)
	{
		select(); // selects function using LED interruptions
	} // end infinite loop
}
