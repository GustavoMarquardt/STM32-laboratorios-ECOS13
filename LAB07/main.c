#include "STM32F10x.h"
#include <cmsis_os.h>
#include <stdbool.h>

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

// Configuração dos pinos do display LCD
#define LCD_RS_PIN 2
#define LCD_EN_PIN 3
#define LCD_DATA_PIN_1 4
#define LCD_DATA_PIN_2 5
#define LCD_DATA_PIN_3 6
#define LCD_DATA_PIN_4 7



int val01_delay = 1000000;
int val02_delay = 1000000;

typedef void(*CommandFunction)(unsigned char command);
typedef void (*DataFunction)(unsigned char data);

// Estrutura do driver do LCD
typedef struct {
 CommandFunction sendCommand;
 DataFunction sendData;
} LCDDriver;


void delay_ms(uint16_t t);
void delay_ms(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6000; l++)
		{
		}
}

void lcd_displayOn(LCDDriver* driver);
void lcd_displayOn(LCDDriver* driver){
	driver->sendCommand(0x0e);
}
void lcd_putValue(unsigned char value);
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

void sendComand(unsigned char command);
void sendComand(unsigned char command){
	// AQUI TAMBÉM TALVEZ
GPIOA->ODR &= ~ (1<<LCD_RS);	/* RS = 0 for command */
	lcd_putValue(command);
}
void sendData(unsigned char data);
void sendData(unsigned char data)
{
	//SE PÁ QUE TEM ERRO AQUI MESMO 
	GPIOA->ODR |= (1<<LCD_RS);	/* RS = 1 for data */
	lcd_putValue(data); 
}

void lcdInit(LCDDriver* driver);
void lcdInit(LCDDriver* driver) {
	
 driver->sendCommand = sendComand;
 driver->sendData = sendData;
}

void lcdClear(LCDDriver* driver);
void lcdClear(LCDDriver* driver) {
 // Enviar o comando para apagar o display LCD
 driver->sendCommand(0x01);
}
void lcdWriteChar(LCDDriver* driver, char character);
void lcdWriteChar(LCDDriver* driver, char character) {
 // Enviar o caractere para o display LCD
 driver->sendData(character);
}
// Função para escrever uma string no display LCD
void lcdWriteString(LCDDriver* driver, const char* string);
void lcdWriteString(LCDDriver* driver, const char* string) {
 // Percorrer a string e enviar cada caractere para o display LCD
//até aqui esta certo essa buceta
 while (*string) {
 lcdWriteChar(driver, *string);
 string++;
 }
 delay_ms(1000);
}

// Função para definir a posição do cursor no display LCD
void lcdSetCursor(LCDDriver* driver, int row, int col);
void lcdSetCursor(LCDDriver* driver, int row, int col) {
 // Calcular o endereço do cursor com base na posição da linha e coluna
 unsigned char address = (0x40 * row) + col;
 // Enviar o comando para posicionar o cursor
 driver->sendCommand(0x80 | address);
}

/*----------------------------------------------------------------------------
  Simple delay routine
 *---------------------------------------------------------------------------*/
void delay (unsigned int count);
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
void setup_RedPill();
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
void ledsON();
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
void ledsOFF();
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


int main(void)
{

	LCDDriver driver;
	osKernelInitialize();
	setup_RedPill();
	lcdInit(&driver);	// initialize CMSIS-RTOS
	osKernelStart(); 
	lcdClear(&driver);
	lcdWriteString(&driver, "testa");
	lcdSetCursor(&driver, 1, 2);
	lcd_displayOn(&driver);
	while(1){
	
	}
}
