#include <stm32f10x.h>

//Function prototypes
void delay_ms(uint16_t t);
void leds_test();
void switches_test();
void buzzer_test();
void buzzer_test2();
void adc_test();

void lcd_init(void);
void lcd_command(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_print(char * str);
void delay_us(uint16_t t);

//Pin definition
//LCD display
#define LCD_RS 15
#define LCD_EN 12

//LEDs
#define LED1 0  //PA0
#define LED2 1  //PA1
#define LED3 2  //PA2
#define LED4 15 //PA15
#define LED5 8  //PA8
#define LED6 6  //PA6
#define LED7 5  //PA5
#define LED8 11 //PA11

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

int main()
{
	//int16_t swa, swb, swc;  //Variables to read the switches according to the port it is connected
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AF clock
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	delay_ms(100);
	
	RCC->APB2ENR |= 0xFC |(1<<9);		//ENABLE clocks for GPIOs and ADC1
	//Setting up outputs for leds
	ADC1->CR2 = 1;	/* ADON = 1 (power-up) */
	ADC1->CR2 |=(1<<2); //enable calibration
	ADC1->SMPR2 = 1<<3; /* SMP1 = 001 */
	delay_us(1);	/* wait 1us to make sure the adc module is stable */
	GPIOA->CRL = 0x43344333;	//PA3, PA4 and PA7: inputs (switches)
	GPIOA->CRH = 0x33333333;  //PA8 - PA15: outputs (leds)
	
	//Settig up inputs for switches
	GPIOB->CRL = 0x4444440B; //PB0 set for output+alternate wave form, since it is connected to buzzer.
	GPIOB->CRH = 0x44444444;
GPIOB->ODR = 0xF000; //set pull-up in PB12 - PB15 	
	GPIOC->CRH = 0x44444444;
	GPIOC->ODR = 0xFFFFFFFF; //set pull-up in GPIOC
	
	delay_ms(1); //wait for I/O setup
	GPIOA->ODR &=~(1<<LCD_RS); //Turn off LED4
	delay_ms(1); //wait for LED4 to turn off
  lcd_init();
  delay_ms(2);
	//Teste dos leds
	unsigned int i;
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	//lcd_data(0x0A);
	//GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Testando leds");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	delay_ms(1);
	for(i=0; i<2; i++)
	{
		leds_test();
	}
	
	//Teste do buzzer
	lcd_command(0x01); //limpa lcd
	lcd_command(0x80); //primeira posição da primeira linha
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Testando buzzer");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	delay_ms(1);
	buzzer_test2();
	
	//Tste das chaves
	lcd_command(0x01); //limpa lcd
	lcd_command(0x80); //primeira posição da primeira linha
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Aperte as teclas");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	switches_test();
		
	//Test for pot + ADC
	lcd_command(0x01); //limpa lcd
	lcd_command(0x80); //primeira posição da primeira linha
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Gire o pot.");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	adc_test();
	
	lcd_command(0x01); //limpa lcd
	lcd_command(0x80); //primeira posição da primeira linha
	lcd_print("Teste da Redpill");
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_command(0xC0);
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	lcd_print("Fim de teste!");	
	GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
	
 }
//pot+ADC test
//we need to set autocalibration at begining ADC_CR2=1 
void adc_test()
{
	unsigned int ADC_CTRL;
	uint16_t ADC_VALUE;
	ADC_CTRL = 0x00000000;
	ADC_VALUE=0x0000;
	
	while(ADC_CTRL !=0x00000007)
	{
		ADC1->SQR3 = 9;	/* choose channel 9 as the input */
		ADC1->CR2 = 1;	/* ADON = 1 (start conversion) */
		ADC1->CR2 |= 2; //autocalibration
		delay_us(2);
		while((ADC1->SR&(1<<1)) == 0); /* wait until the EOC flag is set */		
		//lcd_command(0x01); //limpa lcd
	  //lcd_command(0x80); //primeira posição da primeira linha
		//ADC_VALUE = ADC1->DR;
		switch(ADC1->DR)
		{	
			case 0 ... 1365:
			{
				ADC_CTRL |=(1<<0);
				GPIOA->ODR |=(1<<LED1);
				delay_ms(100);
				break;
			}
			case 1366 ... 2730:
			{
				ADC_CTRL |=(1<<1);
				GPIOA->ODR |=(1<<LED2);
				delay_ms(100);
				break;	
			}
			case 2731 ... 4096:
			{
				ADC_CTRL |=(1<<2);
				GPIOA->ODR |=(1<<LED3);
				delay_ms(100);
				break;	
			}	
			default:
			{
				GPIOA->ODR = 0x00000000;
				break;
			}
		}
	}
}	
 
void delay_ms(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6000; l++)
		{
		}
}

//Test for LEDs
void leds_test()
{
//turning on one-by-one	
		  GPIOA->ODR |= (1<<LED1);	 // led 1 - PA0
			delay_ms(200);
			GPIOA->ODR |= (1<<LED2);  // led 2 - PA1
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LED3);  // led 3 - PA2
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LCD_RS); // led 4 - PA15
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LED5);  // led 5 - PA8
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LED6);  // led 6 - PA6
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LED7);  // led 7 - PA5
		  delay_ms(200);
		  GPIOA->ODR |= (1<<LED8); // led 8 - PA11
		  delay_ms(200);

//turning off one-by-one	
			GPIOA->ODR &= ~(1<<LED1);	 // led 1
			delay_ms(200);
			GPIOA->ODR &= ~(1<<LED2);  // led 3
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LED3);  // led 3
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LCD_RS); // led 4
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LED5);  // led 5
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LED6);  // led 6
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LED7);  // led 7
			delay_ms(200);
		  GPIOA->ODR &= ~(1<<LED8); // led 8
			delay_ms(200);
}	

void switches_test()
{
	long int SW_CTRL;
	SW_CTRL = 0x00000000;
		while(SW_CTRL != 0x0001FFFF)
		{
			GPIOA->ODR = 0x00000000; //Turn off all LEDs
		//switch 1
		if((GPIOB->IDR&(1<<SW1))!=0) //if sw1 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
			//delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0x01); //limpa lcd
			lcd_command(0x80); //primeira posição da primeira linha
			lcd_print("Teste da Redpill");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW01");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<0);
		}	
		//switch 2		
		if((GPIOB->IDR&(1<<SW2))!=0) //if sw2 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW02");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<1);
		}
		//switch 3
		if((GPIOB->IDR&(1<<SW3))!=0) //if sw3 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW03");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<2);
		}
		//switch 4
		if((GPIOB->IDR&(1<<SW4))!=0) //if sw4 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW04");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<3);
		}
		//switch 5
		if((GPIOB->IDR&(1<<SW5))!=0) //if sw5 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW05");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<4);
		}
		//switch 6
		if((GPIOB->IDR&(1<<SW6))!=0) //if sw6 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW06");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<5);
		}
		//switch 7
		if((GPIOB->IDR&(1<<SW7))!=0) //if sw7 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
	    //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW07");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<6);			
		}
		//switch 8
		if((GPIOA->IDR&(1<<SW8))!=0) //if sw8 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW08");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<7);
		}
		//switch 9
		if((GPIOA->IDR&(1<<SW9))!=0) //if sw9 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW09");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<8);
		}	
		//switch 10
		if((GPIOB->IDR&(1<<SW10))!=0) //if sw10 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW10");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<9);
		}	
		//switch 11
		if((GPIOB->IDR&(1<<SW11))!=0) //if sw11 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW11");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<10);
		}	
		//switch 12
		if((GPIOB->IDR&(1<<SW12))!=0) //if sw12 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW12");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<11);
		}	
		//switch 13
		if((GPIOB->IDR&(1<<SW13))!=0) //if sw13 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW13");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<12);
		}	
		//switch 14
		if((GPIOA->IDR&(1<<SW14))!=0) //if sw14 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{	
		  //GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW14");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<13);
	  }
		//switch 15
		if((GPIOC->IDR&(1<<SW15))!=0) //if sw15 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW15");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<14);
		}	
		//switch 16
		if((GPIOC->IDR&(1<<SW16))!=0) //if sw16 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW16");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<15);
		}	
		//switch 17
		if((GPIOC->IDR&(1<<SW17))!=0) //if sw17 is high
			GPIOA->ODR &= ~(1<<0);
		else
		{
			//GPIOA->ODR |= (1<<0);
		  //delay_ms(1000);
			//GPIOA->ODR &= ~(1<<0);
			lcd_command(0xC0);
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			lcd_print("SW17");
			GPIOA->BRR |=(1<<LCD_RS)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8); //Turn off LEDs used in LCD
			SW_CTRL |=(1<<16);
		}	
	}		
}
//Test the buzzer
void buzzer_test2()
{
	
			RCC->APB1ENR |= (1<<1);
      TIM3->CCR2 = 5000;
			TIM3->CCER = 0x1 << 8; /*CC2P = 0, CC2E = 1 */
			TIM3->CCMR2 = 0x0030;  /* toggle channel 3 */
			TIM3->ARR = 10000-1;				
			TIM3->CR1 = 1;
      delay_ms(3000);	
			RCC->APB1ENR &= ~(1<<1);
}	
void buzzer_test()
{
	if((GPIOA->IDR&(1<<3))!=0) //if sw8 is high
		{
			//TIM3->CR1 = 0;
		  RCC->APB1ENR &= ~(1<<1);
		}	
		else
		{
			delay_ms(300); //debouncing
			if ((GPIOA->IDR&(1<<3))!=1)
			{
			RCC->APB1ENR |= (1<<1);
      TIM3->CCR2 = 10000;
			TIM3->CCER = 0x1 << 8; /*CC2P = 0, CC2E = 1 */
			TIM3->CCMR2 = 0x0030;  /* toggle channel 3 */
			TIM3->ARR = 10000-1;				
			TIM3->CR1 = 1;
			//delay_ms(2000);
			//TIM3->CR1 = 0;
			
			}
			else
			{
				delay_ms(300);
				if ((GPIOA->IDR&(1<<3))!=0)
				//TIM3->CR1 = 0;
			  RCC->APB1ENR &= ~(1<<1);
			}	
		}
}

void lcd_putValue(unsigned char value)
{
	uint16_t aux;
	aux = 0x0000; //clear aux
	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	//GPIOA->BSRR = (value>>4)&0x0F; /* put high nibble on PA0-PA3 */	
	aux = value & 0xF0;
	aux = aux>>4;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	//GPIOA->BSRR = ;
	//GPIOA->BSRR = ;
	//GPIOA->BSRR = ;
	
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	delay_ms(3);			/* make EN pulse wider */
	GPIOA->ODR &= ~ (1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
	delay_ms(1);			/* wait	*/

	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	//GPIOA->BSRR = value&0x0F; /* put low nibble on PA0-PA3 */	
	aux = 0x0000; //clear aux
	aux = value & 0x0F;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	//GPIOA->BSRR = (aux&0x0008)<<8;
	//GPIOA->BSRR = (aux&0x0004)<<6;
	//GPIOA->BSRR = (aux&0x0002)<<5;
	//GPIOA->BSRR = (aux&0x0001)<<5;
	//GPIOA->ODR = aux;
	
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	delay_ms(3);			/* make EN pulse wider */
  GPIOA->ODR &= ~(1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
  delay_ms(1);			/* wait	*/
}

void lcd_command(unsigned char cmd)
{
	GPIOA->ODR &= ~ (1<<LCD_RS);	/* RS = 0 for command */
	lcd_putValue(cmd);
}

void lcd_data(unsigned char data)
{
	GPIOA->ODR |= (1<<LCD_RS);	/* RS = 1 for data */
	lcd_putValue(data); 
}

void lcd_print(char * str)
{
  unsigned char i = 0;

	while(str[i] != 0) /* while it is not end of string */
	{
		lcd_data(str[i]); /* show str[i] on the LCD */
		i++;
	}
}

void lcd_init()
{
	delay_ms(15);
	GPIOA->ODR &= ~(1<<LCD_EN);	/* LCD_EN = 0 */
	delay_ms(3); 			/* wait 3ms */
	lcd_command(0x33); //lcd init.
	delay_ms(5);
	lcd_command(0x32); //lcd init.
	delay_us(3000);
	lcd_command(0x28); // 4-bit mode, 1 line and 5x8 charactere set
	delay_ms(3);
	lcd_command(0x0e); // display on, cursor on
	delay_ms(3);
	lcd_command(0x01); // display clear
	delay_ms(3);
	lcd_command(0x06); // move right
	delay_ms(3);
}

void delay_us(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6; l++)
		{
		}
}
