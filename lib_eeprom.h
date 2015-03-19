////////////////////////////////////////////////////////////
//
//	Title:		EEPROM Library
//	Author:		Claira Safi
//	Revised:	1/31/15
//	Device:		PIC24F
//
////////////////////////////////////////////////////////////

#ifndef LIB_EEPROM_H
#define	LIB_EEPROM_H

#include "p24f32ka302.h"

#define EEPROM_ADD_START 0x7FFE00
#define EEPROM_ADD_STOP 0x7FFFFF
#define EEPROM_SIZE_WORDS EEPROM_ADD_STOP-EEPROM_ADD_START
#define EEPROM_SIZE_BYTES EEPROM_SIZE_WORDS*2

void eeprom_init()
{
    NVMCONbits.WREN = 1;
    NVMCONbits.PGMONLY = 0;	// Enable auto erase-before-write
}

unsigned int eeprom_read(unsigned int address)
{
    while(NVMCONbits.WR);      // wait for any last write to complete
    TBLPAG = 0x7f;            // eprom is a t 0x7ffe00
    return __builtin_tblrdl(0xFE00+address);
}

void eeprom_write(unsigned int address, unsigned int data)
{
    while(NVMCONbits.WR);      // wait for last write to complete
    NVMCON = 0x4058;        // Set up NVMCON to erase one word of data EEPROM
    TBLPAG = 0x7f;            // eprom is a t 0x7ffe00
    __builtin_tblwtl(0xFE00+address, 0xffff);
    asm volatile ("disi #5");
    __builtin_write_NVM();       // Issue Unlock Sequence & Start Write Cycle

    while(NVMCONbits.WR);      // wait for errase to complete
    NVMCON = 0x4004;        // Set up NVMCON to write one word of data EEPROM
    __builtin_tblwtl(0xFE00+address, data);   // Write EEPROM data to write latch
    asm volatile ("disi #5");
    __builtin_write_NVM();
}

#endif	

