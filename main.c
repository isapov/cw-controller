#define _XTAL_FREQ 20000000

#include <xc.h>

// BEGIN CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG

#define bitset(var,bitno) ((var) |= 1 << (bitno))
#define bitclr(var,bitno) ((var) &= ~(1 << (bitno)))


#define	E                   RA5	
#define	RW                  RA3	
#define	RS                  RA2	

#define	CLR_DISP            0x01	
#define	DISP_ON             0x0C	
#define	DISP_OFF            0x08	
#define	CUR_HOME            0x02
#define	CUR_OFF             0x0C
#define CUR_ON_UNDER        0x0E
#define	CUR_ON_BLINK        0x0F
#define CUR_LEFT            0x10
#define	CUR_RIGHT           0x14
#define	CUR_UP              0x80	
#define	CUR_DOWN            0xC0
#define	ENTER               0xC0
#define	DD_RAM_ADDR         0x80	
#define	DD_RAM_ADDR2        0xC0	

#define	COL1                RB0
#define	COL2                RB1
#define	COL3                RB2
#define	LED                 RB4
#define	BUZZ1               RC4
#define	BUZZ2               RC5
#define	RELAY               RA1

#define	OFF                 0
#define	ON                  1

#define LED_OFF             1
#define LED_ON              0

#define BUTTON_NONE         0
#define BUTTON1_ON      	1
#define BUTTON1_OFF         2
#define BUTTON2_ON          3
#define BUTTON2_OFF         4
#define BUTTON3_ON          5
#define BUTTON3_OFF         6
#define BUTTON4_ON          7
#define BUTTON4_OFF         8
#define BUTTON5_ON          9
#define BUTTON5_OFF         10
#define BUTTON6_ON          11
#define BUTTON6_OFF         12
#define BUTTON_LONGPRESS    128

#define IS_RELAY_ON         (relayOn == ON)
#define IS_RELAY_OFF        (relayOn == OFF)

#define SET_RELAY_ON()      do{ relayOn = ON; RELAY = ON; } while(0)
#define SET_RELAY_OFF()     do{ relayOn = OFF; RELAY = OFF; } while(0)

unsigned char B1, B2, B3, B4, B5, B6;
unsigned int B1cnt, B2cnt, B3cnt, B4cnt, B5cnt, B6cnt;

unsigned int longpressed;

void E_Pulse(void) {
	asm("nop");
    asm("nop");
    E=1;
	asm("nop");
	asm("nop");
	E=0;
}

unsigned char OlimexMsg[] = " www.olimex.com ";
unsigned char data;

// Init LCD  after reset
void InitLCD(void) {

	INTCON 	= 0x0;			// Disable inerupt
	CMCON 	= 0x07;		    // Comparators Off
	ADCON1  = 0x06;			// Port as Digital IO
	TRISA2 	= 0;			// RS pin as output							
	TRISA3 	= 0;			// RW pin as output							
	TRISA5 	= 0;			// E pin as output	
	TRISC	= 0xF0;			// D4-D7 as output
						

	RS=0;				
	RW=0;
	__delay_ms(110);	
	PORTC=0b00000011;		
	E_Pulse();
	__delay_ms(10);	
	PORTC=0b00000011;
	E_Pulse();
	__delay_ms(10);
	PORTC=0b00000011;
	E_Pulse();
	__delay_ms(10);
	PORTC=0b00000010;
	E_Pulse();
}

// Send char to LCD
LCDSendChar(unsigned char c) {
	
	__delay_ms(2);
	//get upper nibble
	data = c & 0b11110000;		
	//set D4-D7	
	data = data >> 4;			
	//send data to LCD
	PORTC = data;				
	//set LCD to write
	RW=0;						
	//set LCD to data mode
	RS=1;						
	//toggle E for LCD
	E_Pulse();
	// get lower nibble
	data = c & 0b00001111;
	//send data to LCD
	PORTC = data;
	//set LCD to write
	RW=0;			
	//set LCD to data mode			
	RS=1;						
	//toggle E for LCD
	E_Pulse();
}

// Send command to LCD
LCDSendCmd(unsigned char c) {

	__delay_ms(2);
	//get upper nibble
	data = c & 0b11110000;		
	//set D4-D7	
	data = data >> 4;			
	//send data to LCD
	PORTC = data;				
	//set LCD to write
	RW=0;						
	//set LCD to data mode
	RS=0;						
	//toggle E for LCD
	E_Pulse();
	// get lower nibble
	data = c & 0b00001111;
	PORTC = data;
	//set LCD to write
	RW=0;						
	//set LCD to data mode
	RS=0;						
	//toggle E for LCD
	E_Pulse();
}

void LCDSendStr(unsigned char* str) {

	unsigned int i=0;
	while(str[i]) {
		LCDSendChar(str[i]);
		i++;
	}
}

unsigned char ButtonScan(void) {

	//set COL1 ---------------------------------------------
	PORTB |= 0x07;
	TRISB |= 0x07;
	bitclr(TRISB, 0);	 						
	__delay_ms(2);
	
	//button 1 is pressed		
	if((COL2==1)&&(B1==OFF)) {
        B1=ON;
        return BUTTON1_ON;
    }
    if((COL2==1)&&(B1==ON)) {
        B1=ON;
        if(B1cnt > 100) {
            longpressed = 1;
            B1cnt=0;
            return BUTTON1_ON | BUTTON_LONGPRESS;
        }
        else
            B1cnt++;
    }
	if((COL2==0)&&(B1==ON))  {
        B1=OFF;
        B1cnt = 0;
        if(longpressed > 0) {
            longpressed = 0;
            return BUTTON1_OFF | BUTTON_LONGPRESS;
        }
        return BUTTON1_OFF;
    }
	
	//button 6 is pressed
	if((COL3==1)&&(B6==OFF)) { B6=ON; return BUTTON6_ON; }	
	if((COL3==0)&&(B6==ON))  { B6=OFF; return BUTTON6_OFF; }
	
	//set COL1 ---------------------------------------------
	PORTB |= 0x07;
	TRISB |= 0x07;
	bitclr(TRISB,1);								
	__delay_ms(2);
	
	//button 4 is pressed
	if((COL1==1)&&(B4==OFF)) { B4=ON; return BUTTON4_ON; }
	if((COL1==0)&&(B4==ON))  { B4=OFF; return BUTTON4_OFF; }			
	
	//button 2 is pressed
	if((COL3==1)&&(B2==OFF)) { B2=ON; return BUTTON2_ON; }
	if((COL3==0)&&(B2==ON))  { B2=OFF; return BUTTON2_OFF; }
	
	//set COL1 ---------------------------------------------
	PORTB |= 0x07;
	TRISB |= 0x03;
	bitclr(TRISB,2);							
	__delay_ms(2);
	
	//button 3 is pressed
	if((COL1==1)&&(B3==OFF)) { B3=ON; return BUTTON3_ON; }
	if ((COL1==0)&&(B3==ON)) { B3=OFF; return BUTTON3_OFF; }
	
	//button 5 is pressed
	if((COL2==1)&&(B5==OFF)) {
        B5=ON;
        return BUTTON1_ON;
    }
    if((COL2==1)&&(B5==ON)) {
        B5=ON;
        if(B5cnt > 100) {
            longpressed = 1;
            B5cnt=0;
            return BUTTON5_ON | BUTTON_LONGPRESS;
        }
        else
            B5cnt++;
    }
	if((COL2==0)&&(B5==ON))  {
        B5=OFF;
        B5cnt = 0;
        if(longpressed > 0) {
            longpressed = 0;
            return BUTTON5_OFF | BUTTON_LONGPRESS;
        }
        return BUTTON5_OFF;
    }

	return BUTTON_NONE;	
}

void Beep1(void) {

	unsigned char t=150;

	while(--t) {
		BUZZ2 = OFF;
		BUZZ1 = ON;
        __delay_us(125);
		BUZZ1 = OFF;
		BUZZ2 = ON;
	 	__delay_us(125);
	}
}

void Beep2(void) {

	unsigned char t=75;

	while(--t) {
		BUZZ2 = OFF;
		BUZZ1 = ON;					
		__delay_us(250);
		BUZZ1 = OFF;
		BUZZ2 = ON;
	 	__delay_us(250);
	}
}

int itoa(int value, char *sp, int radix)
{
    char tmp[16];// be careful with the length of the buffer
    char *tp = tmp;
    int i;
    unsigned v;

    int sign = (radix == 10 && value < 0);    
    if (sign)
        v = -value;
    else
        v = (unsigned)value;

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix; // v/=radix uses less CPU clocks than v=v/radix does
        if (i < 10)
          *tp++ = i+'0';
        else
          *tp++ = i + 'a' - 10;
    }

    int len = tp - tmp;

    if (sign) 
    {
        *sp++ = '-';
        len++;
    }

    while (tp > tmp)
        *sp++ = *--tp;

    return len;
}

void initPorts()
{
    // PORTA - RELAY
	INTCON 	= 0x0;			// Disable inerupt
	CMCON 	= 0x07;		    // Comparators Off
	ADCON1  = 0x06;			// Port as Digital IO
	TRISA1 	= 0;			// RELAY pin as output
    RELAY=OFF;

	// PORTB - LED, BUTTONS
	INTCON 	= 0x00;		// Disable inerupt
	TRISB 	= 0xEF;     // All as input, except LED pin
	LED = LED_OFF;
}

void initGlobals()
{
    longpressed = 0;

	// Clear variable
	B1=B2=B3=B4=B5=B6=0;
    B1cnt=B2cnt=B3cnt=B4cnt=B5cnt=B6cnt=0;
}

// main function
void main(void) {
    
    unsigned char but;
    int c = 50;
    char int_str[30]; // be careful with the length of the buffer
    int len = 0;
    int relayOn = OFF;
    int savedC = c;
    int forcedToStop = 0;
    
    initPorts();
    initGlobals();

    InitLCD();
	LCDSendCmd(DISP_ON);
	LCDSendCmd(CLR_DISP);
    len = itoa(c, int_str, 10);
    int_str[len] = 0;
    LCDSendStr(int_str);
 
	// loop forever 
	while(1) {

		// Scan Button
	 	but = ButtonScan();
	
		if(but == BUTTON1_ON) {
            if(IS_RELAY_OFF)
                LED = LED_ON;
        }
        else if(but == (BUTTON1_ON | BUTTON_LONGPRESS)) {
            if(IS_RELAY_OFF) {
                c = c + 10;
                forcedToStop = 0;
            }
        }
        else if(but == (BUTTON1_OFF | BUTTON_LONGPRESS)) {
            if(IS_RELAY_OFF)
                LED = LED_OFF;
        }
		else if(but == BUTTON1_OFF) {
            if(IS_RELAY_OFF) {
                forcedToStop = 0;
                c++;
                LED = LED_OFF;
            }
        }
		else if(but == BUTTON2_ON)  	{
            if(c > 0) {
                if(IS_RELAY_ON) {
                    forcedToStop = 1; //pause
                    SET_RELAY_OFF();
                    LED = LED_OFF;
                }
                else {
                    if(!forcedToStop) {
                        savedC = c;
                    }
                    SET_RELAY_ON();
                    LED = LED_ON;
                }
            }
        }
		else if(but == BUTTON2_OFF) 	{  }
        else if(but == BUTTON3_ON) {
            savedC = c;
        }
        else if(but == BUTTON5_ON) 		{
            if(IS_RELAY_OFF)
                LED = LED_ON;
        }
        else if(but == (BUTTON5_ON | BUTTON_LONGPRESS)) {
            if(IS_RELAY_OFF) {
                c = c-10;
                forcedToStop = 0;
                if(c < 0)
                    c = 0;
            }
        }
        else if(but == (BUTTON5_OFF | BUTTON_LONGPRESS)) {
            if(IS_RELAY_OFF)
                LED = LED_OFF;
        }
		else if(but == BUTTON5_OFF) 	{
            if(IS_RELAY_OFF) {
                c--;
                if(c < 0) c=0;
                forcedToStop = 0;
                LED = LED_OFF;
            }
        }
        else if(but == BUTTON6_ON) 	{
            if(IS_RELAY_ON) {
                c--;         //count revolutions down
                if(c <= 0) { //if the count reached zero
                    forcedToStop = 0;
                    c = savedC;  //reset the counter
                    LED = LED_OFF;
                    SET_RELAY_OFF();
                }
            }
        }
        if(but != BUTTON_NONE) {
            LCDSendCmd(CLR_DISP);
            len = itoa(c, int_str, 10);
            int_str[len] = 0;
            LCDSendStr(int_str);
        }
	}
}