#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/boot.h>

#include "twi_master/twi_master.h"
#include "flash.h"
#include "crc32.h"
#include "bootloader.h"

#define F_CPU 16000000UL

//#define RESET_VECTOR 0x0000
//#define RESET_VECTOR_ARGUMENT_ADDRESS 0x0002

uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

//static uint16_t jmp_instruction = 0x940c;

typedef int bool;
#define true  1
#define false 0

uint16_t mem_addr = 0x0030; //firmware start address (application_start_address_byte_offset)
uint16_t mem_addr1 = 0x0030;

volatile uint8_t ic_addr = 0x50;
volatile uint16_t application_crc_expected_index = 28;

static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;

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

static inline void writeToFlash(const uint16_t address, uint8_t *data){ //,uint16_t application_start) {

/*  if (0 == address) {
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
*/
  writeToPageBuffer(address, data);
  writePageBufferToFlash(address);
}

static inline void writeFlashFromI2C(const uint8_t i2c_address){ //,uint16_t application_start) {
  
  uint16_t start_address = mem_addr1;
  uint16_t length = twi_master_read16_reg16(ic_addr, 32);
  uint8_t buf[SPM_PAGESIZE];
  uint8_t writes = 0;

  for (uint16_t pos = 0; pos < length; pos += 2) {
    if (pos > 0 && (0 == (pos % SPM_PAGESIZE))) {
      writeToFlash(writes * SPM_PAGESIZE, &buf[0]); //, application_start);
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

  writeToFlash(writes * SPM_PAGESIZE, &buf[0]); //, application_start);
}

int main(void) 
{

  uint16_t application_start = 0;
  uint8_t dc =0;
  uint8_t crcc = 0;
  twi_master_init();

  /*
  MCUSR &= ~(1<<WDRF);
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 0x00;
  */
  
  DDRB |= (1<<1);
  PORTB |= (1<<1);
  
  dc = twi_master_read16_reg8(ic_addr, 26);
  crcc = isCrcOk(ic_addr);
  
  if (dc == 0x11 && crcc == 1)
  {
    twi_master_write16_reg8(ic_addr, 26, 0xFF);
    eraseApplication();
    writeFlashFromI2C(ic_addr); //, application_start);
  }
  /*else {
    uint16_t address_in_external_eeprom = getWordFromSource(
        source_i2c_address_for_program, application_start_address_byte_offset);

    application_start = address_in_external_eeprom >> 8;
    application_start |= static_cast<uint8_t>(address_in_external_eeprom);
  }*/
  jump_to_app();
}
