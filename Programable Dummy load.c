/*
 * Programable_Dummy_load.c
 *
 * Created: 7.2.2015. 20:21:23
 *  Author: akosanovic
 */ 
// define some macros
#define F_CPU 16000000ul
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR
#define DEBUGEN (PIND & (1<<PD5))
#define IRANGE (PIND & (1<<PD6))				//when PD6 low IRANGE = 0
#define DAC_ADR  0x60   //address of i2c DAC

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include <stdio.h>
#include <avr/pgmspace.h>

#include "lcdpcf8574.h"



void spi_init_master (void);
void initInterrupts(void);
void uart_init (void);
void adc_init(void);
uint16_t adc_read(uint8_t);
void uart_transmit (unsigned char);
void writeString(char*);
//unsigned char spi_tranceiver (unsigned char);
void spi_tranceiver (unsigned char);
void setRange (int, uint8_t);
//volatile uint16_t enc = 0;

volatile uint16_t enc = 0;
volatile uint8_t  update = 1;

int main(void)
{
	initInterrupts();
	uart_init();
	adc_init();
	_delay_ms(20);
	spi_init_master();
	_delay_ms(20);
	i2c_init();                             // initialize I2C library
	sei();					// turn on interrupts
	//init lcd
	lcd_init(LCD_DISP_ON);

	//lcd go home
	lcd_home();
	lcd_led(0);
	uint16_t adc_result0;
	uint16_t adc_result1;
	uint16_t adc_result2;
	uint16_t uinput = 0;
	uint8_t updatelcd=0;
	char int_buffer[10];
	char data[2];
	char enc_buffer[10];
	DDRD &= ~(1 << PD5);	//set PD5 input (DEBUG)
	PORTD |= (1<<PD5);		//set internal pull-up
	char str[10]="hello\n";
	writeString(str);
	
	//initialize DAC
	i2c_start_wait((DAC_ADR<<1)|I2C_WRITE);     // set device address and write mode
	i2c_write(0b10011000);
	i2c_stop();                             // set stop conditon = release bus
	_delay_ms(50);
	
	//Initialize DPOT1 set OpAMp to gain 2
	setRange(0, 0x0F);
	uint8_t igain = 2;
	_delay_ms(20);

	//Initialize DPOT2 set divider to 0k, larger range, divider 10/230= 1/23 23times
	setRange(1, 0x01);
	uint8_t vrange = 2;

    while(1)
    {
		
	if (IRANGE && igain == 2){
		setRange(0, 230);
		igain = 10;
		}
	else if ((!IRANGE) && igain == 10){
			setRange(0, 0x0F);
			igain = 2;
		}
	{
	}
	if (update){
		data[0] =  ((enc>> 6) & 0x0F);
		data[1] =  (enc<<2);
		i2c_start_wait((DAC_ADR<<1)|I2C_WRITE);     // set device address and write mode
		i2c_write(data[0]);
		i2c_write(data[1]);                      
		i2c_stop();                             // set stop conditon = release bus
		update = 0;
	 }
	adc_result0 = adc_read(0);
	_delay_ms(50);
	adc_result1 = adc_read(1);
	if (adc_result1 < 500 && vrange == 2){
		setRange(1,0xFE);				//DPOT2 set divider to 10k, smaller range, divider 20/240= 1/12 12times
		vrange = 1;
		}
	else if (adc_result1 > 1020 && vrange == 1){
		setRange(1,0x01);				//DPOT2 set divider to 0k, larger range, divider 10/230= 1/23 23times
		vrange = 2;		
		}
	_delay_ms(50);
	adc_result2 = adc_read(2);     // read adc value 
	//calculation of voltage
	if (vrange ==1){
		uinput=adc_result1*24;}
	else if (vrange ==2){
		uinput=adc_result1*46;}

	if (DEBUGEN){							//if debug send data over serial
		itoa(adc_result0,int_buffer,10);
		writeString("ADC_I:");
		writeString(int_buffer);
		uart_transmit('\t');
		writeString("Gain_I:");
		itoa(igain,int_buffer,10);
		writeString(int_buffer);
		uart_transmit('\t');
		writeString("ADC_V:");
		itoa(uinput,int_buffer,10);
		writeString(int_buffer);
		writeString("mV");
		uart_transmit('\t');
		writeString("range_V:");
		itoa(vrange,int_buffer,10);
		writeString(int_buffer);
		uart_transmit('\t');
		writeString("ADC_S:");
		itoa((adc_result2*2),int_buffer,10);
		writeString(int_buffer);
		writeString("mV");
		uart_transmit('\t');
		writeString("enc:");
		itoa(enc,enc_buffer,10);
		writeString(enc_buffer); 
		uart_transmit('\n');
		}
		
		if (updatelcd==5){
				lcd_clrscr();
		    	itoa(uinput,int_buffer,10);
		    	//lcd_gotoxy(0, 0);
		    	lcd_puts("Vin= ");
		    	lcd_gotoxy(5, 0);
		    	lcd_puts(int_buffer);
		    	lcd_gotoxy(0, 1);
		    	lcd_puts("Iset= ");
		    	lcd_gotoxy(6, 1);
				itoa((enc*2),int_buffer,10);
		    	lcd_puts(int_buffer);
				lcd_gotoxy(11, 1);
				if (igain == 2){
					itoa((adc_result2*2),int_buffer,10);
				}
				else if (igain == 10){
					itoa((adc_result2*10),int_buffer,10);
				}
				lcd_puts(int_buffer);
				updatelcd=0;
				}
		updatelcd++;
    }
}
// Routine to setup INT0
void initInterrupts(void)
{
	// Assure that pin PB2 (INT0) and PB0 are inputs
	DDRD &= ~(1<<PD2);
	DDRD &= ~(1<<PD3);
	DDRD &= ~(1<<PD4);
	PORTD |= (1<<PD2)|(1<<PD3)|(1<<PD4);// Enable the pull-up resistors
	EICRA |= (1<<ISC01);	// Falling edge in INT0 (PD2) to cause interrupt
	EIMSK |= (1 << INT0);	// Enable and INT0
	EICRA |= (1<<ISC11);	// Falling edge in INT1 (PD3) to cause interrupt
	EIMSK |= (1 << INT1);	// Enable and INT1
}

// The Interrupt Service Routine for external INT0
ISR(INT0_vect)
{
	// When an interrupt occurs, we only have to check the level of
	// of pin PD4 to determine the direction
	if (PIND & _BV(PD4)){
		// Increase enc
		if (enc<1023){
			enc++;
			update = 1;
		}
	}
	else{
		if (enc>0){
			// Decrease enc
			enc--;
			update = 1;
		}
	}
}

ISR(INT1_vect)
{
	enc = 0;
	update = 1;
}

// function to initialize ADC
void adc_init()
{
	// AREF = external Vref
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
uint16_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with ’7? will always keep the value
	// of ‘ch’ between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	
	// start single convertion
	// write ’1? to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes ’0? again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}

// function to initialize UART
void uart_init (void)
{
	UBRR0H=(BAUDRATE>>8);
	UBRR0L=BAUDRATE;                        //set baud rate
	UCSR0B|=(1<<TXEN0)|(1<<RXEN0);          //enable receiver and transmitter
	UCSR0C|=(1<<UCSZ01)|(1<<UCSZ00);		// 8bit data format
}
// function to send data over UART
void uart_transmit (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}

//Receive a character through UART
unsigned char UART0_Rx_Char()
{
	//wait for the character
	while(!(UCSR0A & (1<<RXC0)));

	//return the received character
	return(UDR0);
}

//Receive string through UART
unsigned char * UART0_Rx_Str()
{
	unsigned char string[20], x, i = 0;

	//receive the characters until ENTER is pressed (ASCII for ENTER = 13)
	while((x = UART0_Rx_Char()) != 13)
	{
		//and store the received characters into the array string[] one-by-one
		string[i++] = x;
	}

	//insert NULL to terminate the string
	string[i] = '\0';

	//return the received string
	return(string);
}
/*! \brief Writes an ASCII string to the TX buffer */
void writeString(char *str)
{
	while (*str != '\0')
	{
		uart_transmit(*str);
		++str;
	}
}

//Initialize SPI Master Device
void spi_init_master (void)
{
	DDRB = (1<<5)|(1<<3)|(1<<PB2);				//Set MOSI, SCK, SS as Output
	DDRB |= (1<<PB2);					//pull-ip on SS
	DDRB &= ~(1<<4);					//Set MISO as Input
	PORTB |= (1<<4);					//Pull-up on MISO
	DDRB |= (1<<PB0);					// DPOT CS1 pin as output set high
	PORTB |= (1<<PB0);
	DDRB |= (1<<PB1);					// DPOT CS2 pin as output set high
	PORTB |= (1<<PB1);			
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); //Enable SPI, Set as Master, Prescaler: Fosc/16
}
//Function to send and receive data over SPI
void spi_tranceiver (unsigned char data)
{
	SPDR = data;                       //Load data into the buffer
	while(!((SPSR)&(1<<SPIF)));
	//return(SPDR);                      //Return received data
}

void setRange (int channel, uint8_t rvalue){
		uint8_t ch = channel;
		uint8_t val = rvalue;
		PORTB &= ~(1<<ch);
		spi_tranceiver(0b00010001);
		spi_tranceiver(val);
		PORTB |= (1<<ch);

}
