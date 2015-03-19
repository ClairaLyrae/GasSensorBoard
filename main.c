////////////////////////////////////////////////////////////
//
//	Title:		Gas Sensor Board
//	Author:		Claira Safi
//	Revised:	1/31/15
//	Device:		PIC24FJ64GA002
//
////////////////////////////////////////////////////////////

// Gas Sensor Board

////////////////////////////////////////////////////////////
//	Includes/Config
////////////////////////////////////////////////////////////

#include "p24f32ka302.h"
#include <i2c.h>
#include <uart.h>
#include <spi.h>
#include "lib_eeprom.h"

 int FBS __attribute__((space(prog), address(0xF80000))) = 0xF ;
 int FGS __attribute__((space(prog), address(0xF80004))) = 0x3 ;
 int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0x42;
 int FOSC __attribute__((space(prog), address(0xF80008))) = 0xDA ;
 int FWDT __attribute__((space(prog), address(0xF8000A))) = 0x5F ;
 int FPOR __attribute__((space(prog), address(0xF8000C))) = 0xFF ;
 int FICD __attribute__((space(prog), address(0xF8000E))) = 0xE3 ;
 int FDS __attribute__((space(prog), address(0xF80010))) = 0xDF ;

////////////////////////////////////////////////////////////
//	Freq Constants
////////////////////////////////////////////////////////////

#define XTFREQ          20000000         	  // On-board Crystal frequency
#define PLLMODE         1               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE        9600
#define BRGVAL          ((FCY/(BAUDRATE*16))-1)

////////////////////////////////////////////////////////////
//	Pin Mapping
////////////////////////////////////////////////////////////

 // LED Debug Pins
#define LED1		_RA4
#define LED1_TRIS	TRISAbits.TRISA4
#define LED2		_RB5
#define LED2_TRIS	TRISBbits.TRISB5
#define LED3		_RB6
#define LED3_TRIS	TRISBbits.TRISB6


// Gas Sensor Heater PWM Pins
#define GS1_HEATER			_RB10
#define GS1_HEATER_TRIS		TRISBbits.TRISB10
#define GS2_HEATER			_RB11
#define GS2_HEATER_TRIS		TRISBbits.TRISB11

 // Gas Sensor ADC Pins
#define GS1_ADC			_RB12
#define GS1_ADC_TRIS	TRISBbits.TRISB12 //AN12
#define GS1_ADC_AN		_ANSB12
#define GS2_ADC			_RB13
#define GS2_ADC_TRIS	TRISBbits.TRISB13 //AN11
#define GS2_ADC_AN		_ANSB13

 // Pressure Sensor ADC Pins
#define PS_POS			_RB14
#define PS_POS_TRIS		TRISBbits.TRISB14
#define PS_POS_AN		_ANSB14
#define PS_NEG			_RB15
#define PS_NEG_TRIS		TRISBbits.TRISB15 //AN9
#define PS_NEG_AN		_ANSB15

// I2C Slave Bus Pins
#define SCL			_RB8
#define SCL_TRIS	TRISBbits.TRISB8
#define SDA			_RB9
#define SDA_TRIS	TRISBbits.TRISB9

// MAX1407 Device Pins
//#define SDI			_RB10
//#define SDI_TRIS	TRISBbits.TRISB10
//#define SCLK		_RB11
//#define SCLK_TRIS	TRISBbits.TRISB11
//#define SS			_RB12
//#define SS_TRIS		TRISBbits.TRISB12
//#define SDO			_RB13
//#define SDO_TRIS	TRISBbits.TRISB13
//#define DRDY		_RA6
//#define DRDY_TRIS	TRISBbits.TRISA6

////////////////////////////////////////////////////////////
//	Constants
////////////////////////////////////////////////////////////

// Slave I2C Address (7-bit)
#define I2C_ADDRESS 0b11101100

////////////////////////////////////////////////////////////
//	Bitfields
////////////////////////////////////////////////////////////

static volatile struct DEVICECON {
	unsigned UNUSED : 8;	// Protocol mode
} DEVICECONbits;

static volatile struct DEVICESTAT {
	unsigned I2CSTATE: 2;
} DEVICESTATbits;

#define I2C_STATE_IDLE	0
#define I2C_STATE_MEM	1
#define I2C_STATE_DATA	2


////////////////////////////////////////////////////////////
//	I2C Register
////////////////////////////////////////////////////////////

// Register contents
#define REG_DEVICESTAT	0x00	// Device status bits
#define REG_DEVICECON	0x01	// Device control bits	(<0:3> , <0> EEPROM config write)
#define REG_GSC1CON		0x02
#define REG_GSC1HTR		0x03	// Catalytic gas sensor heater drive voltage (50mV/bit, 0 to 12.75V)
#define REG_GSC2CON		0x04
#define REG_GSC2HTR		0x05	// Catalytic gas sensor heater drive voltage (50mV/bit, 0 to 12.75V)
#define REG_GSE1CON		0x06
#define REG_GSE1REF		0x07	// Electrochemical gas sensor reference voltage (10mV/bit, signed, -1.28 to 1.28)
#define REG_GSE2CON		0x08
#define REG_GSE2REF		0x09	// Electrochemical gas sensor reference voltage (10mV/bit, signed, -1.28 to 1.28)
#define REG_GSC1RESH	0x0A	// Catalytic gas sensor output (ppm)
#define REG_GSC1RESL	0x0B
#define REG_GSC2RESH	0x0C	// Catalytic gas sensor output (ppm)
#define REG_GSC2RESL	0x0D
#define REG_GSE1RESH	0x0E	// Electrochemical gas sensor output (ppm)
#define REG_GSE1RESL	0x0F
#define REG_GSE2RESH	0x10	// Electrochemical gas sensor output (ppm)
#define REG_GSE2RESL	0x11
#define REG_PSRESH		0x12	// Pressure Sensor Output (kPa)
#define REG_PSRESL		0x13
#define REG_TSRESH		0x14	// Temperature Sensor Output (kPa)
#define REG_TSRESL		0x15

#define REGISTER_ADD_EEPROM	0x00	// Address of register contents as mirrored in EEPROM

// Register for I2C addressing
#define REGISTER_SIZE		0x16
static volatile char register_data[REGISTER_SIZE];
static volatile unsigned register_pointer = 0;

// Register R/W masking bits (1 = writeable, 0 = read only)
static volatile char register_rw_mask[REGISTER_SIZE];
static volatile char register_changed[REGISTER_SIZE];
static volatile unsigned register_if;

// Copies the contents of a char over masked by a given char bits
char inline copyCharMasked(char src, char value, char mask) {
	return (value & mask) | (src & (mask ^ 0xffff));
}

void inline readRegisterFromEEPROM() {
	int i = 0;
	for(i = 0; i < REGISTER_SIZE; i++)
		register_data[i] = copyCharMasked(register_data[i], eeprom_read(EEPROM_ADD_START + i), register_rw_mask[i]);
}

void inline writeRegisterToEEPROM() {
	int i = 0;
	for(i = 0; i < REGISTER_SIZE; i++)
		eeprom_write(EEPROM_ADD_START + i, register_data[i]);
}

// Setup bit R/W masking for registers
void inline initializeRegister() {
	//readRegisterFromEEPROM();
	register_rw_mask[REG_DEVICECON] = 0xff;
	register_rw_mask[REG_GSC1CON] = 0xff;
	register_rw_mask[REG_GSC2CON] = 0xff;
	register_rw_mask[REG_GSE1CON] = 0xff;
	register_rw_mask[REG_GSE2CON] = 0xff;
	register_rw_mask[REG_GSE1REF] = 0xff;
	register_rw_mask[REG_GSE2REF] = 0xff;
	register_rw_mask[REG_GSC1HTR] = 0xff;
	register_rw_mask[REG_GSC2HTR] = 0xff;
}


void inline refreshRegister() {
	// Device configuration register
	if(register_changed[REG_DEVICECON]){
		register_changed[REG_DEVICECON] = 0;
		writeRegisterToEEPROM();
	}
	// Electrochemical Sensor Configuration updates
	if(register_changed[REG_GSE1CON]){
		register_changed[REG_GSE1CON] = 0;
	}
	if(register_changed[REG_GSE2CON]){
		register_changed[REG_GSE2CON] = 0;
	}
	// Electrochemical Sensor Reference Voltage updates
	if(register_changed[REG_GSE1REF]){
		register_changed[REG_GSE1REF] = 0;
	}
	if(register_changed[REG_GSE2REF]){
		register_changed[REG_GSE2REF] = 0;
	}
	// Catalytic Sensor Configuration updates
	if(register_changed[REG_GSC1CON]){
		register_changed[REG_GSC1CON] = 0;
	}
	if(register_changed[REG_GSC2CON]){
		register_changed[REG_GSC2CON] = 0;
	}
	// Catalytic Heater PWM updates
	if(register_changed[REG_GSC1HTR]){
		OC2R = register_data[REG_GSC1HTR];
		register_changed[REG_GSC1HTR] = 0;
	}
	if(register_changed[REG_GSC2HTR]){
		OC3R = register_data[REG_GSC2HTR];
		register_changed[REG_GSC2HTR] = 0;
	}
}


////////////////////////////////////////////////////////////
//	Functions
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
//	Initialization
////////////////////////////////////////////////////////////

// Initialize pin configurations and I/O features
inline void initializeIO()
{
    ANSA = 0x0000;
    ANSB = 0x0000;

	GS1_HEATER_TRIS = 0;
	GS2_HEATER_TRIS = 0;
	GS1_ADC_TRIS = 1;
	GS2_ADC_TRIS = 1;
	PS_POS_TRIS = 1;
	PS_NEG_TRIS = 1;
	SDA_TRIS = 0;
	SCL_TRIS = 0;
	LED1_TRIS = 0;
	LED2_TRIS = 0;
	LED3_TRIS = 0;

	GS1_ADC_AN = 1;
	GS2_ADC_AN = 1;
	PS_POS_AN = 1;
	PS_NEG_AN = 1;

    LATA = 0;
    LATB = 0;
}

// Initialize SFR registers for peripherals
inline void initializeSFR()
{
    CLKDIVbits.RCDIV = 0;

	// ADC
	AD1CON1bits.ADON = 1;		// Enable
	AD1CON1bits.ADSIDL = 0;		// Dont stop in idle
	AD1CON1bits.FORM = 0b00;	// Absolute decimal
	AD1CON2bits.BUFM = 0;		// 16-word buffer (no split)
	AD1CON1bits.SSRC = 0b0111;	// Auto sample every X tad
	AD1CON1bits.ASAM = 1;		// Sample immediately after last conversion
	AD1CON2bits.CSCNA = 1;		// Scan inputs
	AD1CON2bits.SMPI = 0b0011;	// Interrupt every 4 conversions
	AD1CON2bits.ALTS = 0;		// Always use channel A
	AD1CON3bits.ADCS = 0b00111111;	// Tad = 64*Tcy
	AD1CON3bits.SAMC = 0b11111;		// Sample every 31 Tad
	AD1CHSbits.CH0NA = 0b000;	// Use AVss for negative input
	//AD1CHSbits.CH0SA = 0b00000;	// Not needed for scan (auto set)
	AD1CSSL = 0b0001111000000000;
	AD1CSSH = 0b0000000000000000;
	_AD1IF = 0;
	_AD1IE = 1;

	// PWM Drivers (Gas Sensor Heaters)
	OC2CON1bits.OCM = 0b101;	// Enable PWM Edge aligned
	OC2CON1bits.OCTSEL = 0b111;	// Fcy clock source
	OC2CON1bits.TRIGMODE = 0;	//
	OC2CON2bits.SYNCSEL = 0b11111;	// Self-sync
	OC3CON1bits.OCM = 0b101;	// Enable PWM Edge aligned
	OC3CON1bits.OCTSEL = 0b111;	// Fcy clock source
	OC3CON1bits.TRIGMODE = 0;	//
	OC3CON2bits.SYNCSEL = 0b11111;	// Sync to OC2
	OC2RS = 0x80;
	OC3RS = 0x80;
	OC2R = 0x40;
	OC3R = 0x40;

	// Timer 2
    TMR2 = 0;               // Clear timer 2
	T2CONbits.T32 = 0;		// Disable dual-timer
	T2CONbits.TCKPS = 0b00;
	T2CONbits.TON = 1;
    PR2 = 152;           // Interrupt every 250ms
    _T2IF = 0;      // Clear interrupt flag
    _T2IE = 1;      // Set interrupt enable bit

	// I2C1 (Slave Device)
	I2C1CONbits.I2CEN = 1;	// I2C enable
	I2C1CONbits.I2CSIDL = 0;	// I2C stop in idle
	I2C1CONbits.SCLREL = 0;	// I2C clock release
	I2C1CONbits.STREN = 0;	// I2C clock stretch enable
	I2C1CONbits.IPMIEN = 0;	// I2C Peripheral Management
	I2C1CONbits.A10M = 0;	// I2C address size = 7 bit
	I2C1CONbits.SMEN = 0;	// I2C address size = 7 bit
	I2C1CONbits.SMEN = 0;	// I2C SMBUS enable
	I2C1MSK = 0x0000;	// I2C SMBUS enable
	I2C1BRG = 157;
	I2C1ADD = 0x54;
	_SI2C1IF = 0;
	_SI2C1IE = 1;

	// SPI Module (MAX1407 Control)
	//	SPI1CON1bits.CKE = 0;
	//	SPI1CON1bits.CKP = 0;
	//	SPI1CON1bits.DISSCK = 0;
	//	SPI1CON1bits.DISSDO = 0;
	//	SPI1CON1bits.MODE16 = 0;
	//	SPI1CON1bits.MSTEN = 0;
	//	SPI1CON1bits.PPRE = 0;
	//	SPI1CON1bits.SPRE = 0;
	//	SPI1CON1bits.SMP = 0;
	//	SPI1CON1bits.SSEN = 0;
	//	SPI1CON2bits.FRMEN = 0;
	//	SPI1CON2bits.SPIBEN = 0;
	//	SPI1CON2bits.SPIFE = 0;
	//	SPI1CON2bits.SPIFPOL = 0;
	//	SPI1CON2bits.SPIFSD =  0;
	//	SPI1STATbits.SPIEN = 1;
	//	SPI1STATbits.SPISIDL = 1;
	//	SPI1STATbits.SISEL = 1;
	//	_SPI1IE = 1;
	//	_SPI1IF = 0;

	// Timer 1
    TMR1 = 0;               // Clear timer 1
    PR1 = 152;           // Interrupt every 250ms
    _T1IF = 0;      // Clear interrupt flag
    _T1IE = 0;      // Set interrupt enable bit
	T1CONbits.TCKPS = 0b00;
	T1CONbits.TON = 0;

	//	// UART
	//    U1BRG = 63;
	//    U1MODEbits.UARTEN = 1; // 1      // Reset UART to 8-n-1, alt pins, and enable
	//	U1STAbits.UTXEN = 1; // 1		// Enable TX
	//    U1MODEbits.UEN = 0b00;      // UART uses RX/TX pins, no cts/rts
	//    U1MODEbits.STSEL = 0;		// 1 stop bit
	//    U1MODEbits.PDSEL = 0b00;	// 8 bits, no parity
	//	U1STAbits.URXISEL = 0b01;	//
	//	U1MODEbits.BRGH = 0;
	//	_U1RXIE = 1;
	//	_U1TXIE = 0;
	//    _U1RXIF = 0;
}

////////////////////////////////////////////////////////////
//	Main Program
////////////////////////////////////////////////////////////

// Main program loop
int main(void)
{
	initializeIO();
	initializeSFR();
	initializeRegister();
    while(1)
    {
		// Check for register changes and update parameters accordingly
		refreshRegister();
	}
	return 0;
}

////////////////////////////////////////////////////////////
// Interrupts
////////////////////////////////////////////////////////////

// Change Notification
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    _CNIF = 0;      // clear interrupt flag
}

// Change Notification
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
	LED1 ^= 1;
    _T2IF = 0;      // clear interrupt flag
}

// Serial port interrupt (MAX1407)
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
    _SPI1IF = 0;      // clear interrupt flag
}

// ADC interrupt (low res gas sensors, pressure sensor)
void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
	register_data[REG_GSC1RESH] = ADC1BUF0 >> 8;
	register_data[REG_GSC1RESL] = ADC1BUF0 & 0xff;
	register_data[REG_GSC2RESH] = ADC1BUF1 >> 8;
	register_data[REG_GSC2RESL] = ADC1BUF1 & 0xff;
	unsigned int psense = (ADC1BUF2-ADC1BUF3);
	register_data[REG_PSRESH] = psense >> 8;
	register_data[REG_PSRESL] = psense & 0xff;
    _AD1IF = 0;      // clear interrupt flag
}

// Serial port interrupt (I2C Slave command)
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void )
{
    unsigned char temp;
	// Address Matched
	if(!I2C1STATbits.D_A)
	{
		// Reset FSM
		LED1 ^= 1;

		// Enter Read Mode
		if(!I2C1STATbits.R_W)
		{
			temp = I2C1RCV;     //dummy read
			DEVICESTATbits.I2CSTATE = I2C_STATE_MEM;	// Wait for RAM address
		}

		// Enter Write Mode (First byte)
		else if(I2C1STATbits.R_W)
		{
			temp = I2C1RCV;
			I2C1TRN = register_data[register_pointer];      //Read data from RAM & send data to I2C master device
			I2C1CONbits.SCLREL = 1; // Release SCL1 line
			while(I2C1STATbits.TBF);
		}
	}
	// Data bytes
	else
	{
		// Data Read
		if(I2C1STATbits.R_W)
		{
			if(++register_pointer >= REGISTER_SIZE)
				register_pointer = 0;
			temp = I2C1RCV;
			I2C1TRN = register_data[register_pointer];      //Read data from RAM & send data to I2C master device
			I2C1CONbits.SCLREL = 1; // Release SCL1 line
			while(I2C1STATbits.TBF);
		}

		// Data Write
		else
		{
			// First byte is register address
			if(DEVICESTATbits.I2CSTATE == I2C_STATE_MEM)
			{
				DEVICESTATbits.I2CSTATE = I2C_STATE_DATA;	// Allow data transfer
				register_pointer = I2C1RCV;
				if(register_pointer >= REGISTER_SIZE)
					register_pointer = 0;
			}
			// Next byte(s) are values to write into register
			else if(DEVICESTATbits.I2CSTATE == I2C_STATE_DATA)
			{
				// Mask input based on register R/W masking bits and write
				//unsigned char mask = register_rw_mask[register_pointer];
				//register_data[register_pointer] = (I2C1RCV & mask) | (register_data[register_pointer] & (mask ^ 0xffff));

				register_data[register_pointer] = copyCharMasked(register_data[register_pointer], I2C1RCV, register_rw_mask[register_pointer]);
				register_changed[register_pointer] = 1;
				if(++register_pointer >= REGISTER_SIZE)
					register_pointer = 0;
			}
		}
	}
	LED2 = DEVICESTATbits.I2CSTATE;
    _SI2C1IF = 0;
}

































