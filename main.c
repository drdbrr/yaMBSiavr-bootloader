/*
 *  Created: 04.02.2016
 *  Author: Max Brueggemann
 */ 

/*
*	An example project implementing a simple modbus slave device using an
*	ATmega88PA running at 20MHz.
*	Baudrate: 38400, 8 data bits, 1 stop bit, no parity
*	Your busmaster can read/write the following data:
*	coils: 0 to 7
*	discrete inputs: 0 to 7
*	input registers: 0 to 3
*	holding registers: 0 to 3
*/

#include <stdlib.h>
#include <string.h>
#include <math.h> //include libm
#include <avr/pgmspace.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "yaMBSiavr.h"
#include "twi_master/twi_master.h"
#include "flash.h"

#include <avr/boot.h>
#include <avr/eeprom.h>

#include "crc32.h"

typedef int bool;
#define true  1
#define false 0

#define clientAddress 0x06

volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[4];
volatile uint16_t holdingRegisters[20];
volatile uint16_t internalRegisters[10];

uint16_t mem_addr = 0x0030; //firmware start address (application_start_address_byte_offset)
uint16_t mem_addr1 = 0x0030;

volatile uint8_t ic_addr = 0x50;
volatile uint16_t application_crc_expected_index = 28;

void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0);
}

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/
void SetOuts(volatile uint8_t in) {
	PORTD|= (((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
	//PORTB|= (((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	in=~in;
	//PORTB&= ~(((in & (1<<0))<<2) | ((in & (1<<1))) | ((in & (1<<2))>>2));
	PORTD&= ~(((in & (1<<3))<<4) | ((in & (1<<4))<<1) | ((in & (1<<5))<<1));
}

uint8_t ReadIns(void) {
	uint8_t ins=0x00;
	ins|=(PINC&((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)));
	ins|=(((PIND&(1<<4))<<2)|((PIND&(1<<3))<<4));
	return ins;
}

void io_conf(void) { 
	/*
	 Outputs: PB2,PB1,PB0,PD7,PD5,PD6
	 Inputs: PC0, PC1, PC2, PC3, PC4, PC6, PD4, PD3
	*/
	DDRD=0x00;
	//DDRB=0x00;
	DDRC=0x00;
	PORTD=0x00;
	//PORTB=0x00;
	PORTC=0x00;
	PORTD|=(1<<0);
	DDRD |= (1<<2)|(1<<5)|(1<<6)|(1<<7);
	DDRB |= (1<<0)|(1<<1);//|(1<<2)|(1<<3);
}

ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second
    
	modbusTickTimer();
}

void modbusGet(void) {
	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;
			
			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;
			
			case fcReadHoldingRegisters: {
				modbusExchangeRegisters(internalRegisters,0,10);
			}
			break;
			
			case fcReadInputRegisters: {
				modbusExchangeRegisters(inputRegisters,0,4);
			}
			break;
			
			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,0,20);
			}
			break;
			
			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,20);
			}
			break;
			
			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}

static inline bool isCrcOk(const uint8_t i2c_address) {
  bool status = false;
  uint32_t crc = 0;
  uint16_t start_address = mem_addr1;//0x0030;//mem_addr; WWWWWTTTTFFFFFFFFF!?!?!?!??!
  
  uint16_t length = twi_master_read16_reg16(ic_addr, 32);
  
  uint32_t table[crc_table_size];
  init_table(&table[0]);

  for (uint16_t pos = 0; pos < length + 1; pos += 2) {
    if (pos >= length)
      break;

    uint16_t data = twi_master_read16_reg16(ic_addr, pos + start_address);
    if (pos == length - 1)
      data &= 0xFF00;

    uint8_t to_little_endian[2];
    to_little_endian[0] = ((uint8_t)(data >> 8));
    to_little_endian[1] = ((uint8_t)(data));

    crc32((const void *)(&to_little_endian[0]), 2, &table[0],
          &crc);
  }

  uint32_t expected_crc = (uint32_t)(twi_master_read16_reg16(i2c_address, application_crc_expected_index))<< 16;
  
  expected_crc |= (uint32_t)(twi_master_read16_reg16(i2c_address, application_crc_expected_index + 2));

  if (crc == expected_crc) {
    status = true;
  }
  return status;
}
/*
static inline void writeToFlash(const uint16_t address, uint8_t *data,
                                uint16_t application_start) {

  if (0 == address) {
    application_start = (uint8_t)(
        (uint16_t)(data[RESET_VECTOR_ARGUMENT_ADDRESS] << 8));
    application_start |=
        (uint8_t)(data[RESET_VECTOR_ARGUMENT_ADDRESS + 1]);

    data[RESET_VECTOR] = (uint8_t)(jmp_instruction);
    data[RESET_VECTOR + 1] = (uint8_t)(jmp_instruction >> 8);

    data[RESET_VECTOR_ARGUMENT_ADDRESS] = (uint8_t)(
        (uint16_t)(BOOTLOADER_START_ADDRESS) / 2);
    data[RESET_VECTOR_ARGUMENT_ADDRESS + 1] = (uint8_t)((
        (uint16_t)(BOOTLOADER_START_ADDRESS) / 2) >> 8);
  }

  writeToPageBuffer(address, data);
  writePageBufferToFlash(address);
}

static inline void writeFlashFromI2C(const uint8_t i2c_address,
                                     uint16_t application_start) {
  uint16_t start_address = mem_addr1;
  uint16_t length = twi_master_read16_reg16(ic_addr, 32);
  uint8_t buf[SPM_PAGESIZE];
  uint8_t writes = 0;

  for (uint16_t pos = 0; pos < length; pos += 2) {
    if (pos > 0 && (0 == (pos % SPM_PAGESIZE))) {
      writeToFlash(writes * SPM_PAGESIZE, &buf[0], application_start);
      ++writes;
    }
    uint16_t data = twi_master_read16_reg16(i2c_address, pos + start_address);
    buf[pos % SPM_PAGESIZE] = (uint8_t)(data >> 8);
    buf[(pos + 1) % SPM_PAGESIZE] = (uint8_t)(data);
  }

  for (uint16_t pos = SPM_PAGESIZE - ((uint16_t)(writes + 1) *(uint16_t)(SPM_PAGESIZE)) % length; pos < SPM_PAGESIZE; ++pos) {
    buf[pos] = 0xFF; // reset contents, since these bytes were not filled in
                     // this page and have value from previous page
  }

  writeToFlash(writes * SPM_PAGESIZE, &buf[0], application_start);
}

//[[ noreturn ]]
static inline void leaveBootloader(uint16_t application_start) {
  // hold my beer and watch this!
    ((void(*)(void))application_start)();
  //void * void application_start();
  while (1)
    ;
}
*/

void software_reset()
{
  wdt_reset();
  wdt_enable(WDTO_250MS);
  exit (1);  // loop forever
} 

int main(void)
{
    uint8_t i=0;
    
    uint16_t pg_nums = 0;
    
    io_conf();
	sei();
	modbusSetAddress(clientAddress);
	modbusInit();
    wdt_enable(7);
	timer0100us_start();
    twi_master_init();
    
    //Тестовые значения
    internalRegisters[4] = 1;
    twi_master_write16_reg8(ic_addr, 26, 0xFF);
    
    PORTB |= (1<<0);//|(1<<1);
    PORTB &= ~(1<<1);
    
    while(1)
    {
        wdt_reset();
	    modbusGet();
        
        switch(holdingRegisters[0]){
            case 1:
                twi_master_write_page(ic_addr, mem_addr, holdingRegisters[2],holdingRegisters);
                mem_addr=mem_addr+holdingRegisters[2];
                
                //if (holdingRegisters[3] == pg_nums) 
                    //mem_addr = 0x0030;
                break;
            case 2:
                //write firmware length
                twi_master_write16_reg8(ic_addr, 32,holdingRegisters[2]);
                twi_master_write16_reg8(ic_addr, 33,holdingRegisters[3]);
                //write crc32 to eeprom
                twi_master_write16_reg8(ic_addr, 28,holdingRegisters[4]);
                twi_master_write16_reg8(ic_addr, 29,holdingRegisters[5]);
                twi_master_write16_reg8(ic_addr, 30,holdingRegisters[6]);
                twi_master_write16_reg8(ic_addr, 31,holdingRegisters[7]);
                
                //write number of pages
                //twi_master_write16_reg8(ic_addr, 26,holdingRegisters[8]);
                //twi_master_write16_reg8(ic_addr, 27,holdingRegisters[9]);
                //pg_nums = (holdingRegisters[8] << 8);
                //pg_nums = holdingRegisters[9];
                
                i=0;
                break;
            case 3:
                i = isCrcOk(ic_addr);
                mem_addr = 0x0030;//reset memory address counter
                //internalRegisters[5] = isCrcOk(ic_addr);
                //internalRegisters[0] = twi_master_read16_reg16(ic_addr, 32);//length
                //internalRegisters[1] = twi_master_read16_reg16(ic_addr, 28);//MSB crc32
                //internalRegisters[2] = twi_master_read16_reg16(ic_addr, 30);//LSB crc32
                break;
            case 4:
                internalRegisters[5] = i;
                break;
            case 5:
                twi_master_write16_reg8(ic_addr, 26, 0x11);
                break;
            case 6:
                software_reset();
                break;
            default:
            break;
        }
        holdingRegisters[0]=0;
    }
}
