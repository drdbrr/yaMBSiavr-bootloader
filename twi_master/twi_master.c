/*************************************************************************
 * TWI Master library functions for AVR MCU                              *
 *                                                                       *
 * Copyright (C) 2011 by Anton 'TwisteR' Dubniak <twister@tfsoft.org.ua> *
 *                                                                       *
 * This program is free software: you can redistribute it and/or modify  *
 * it under the terms of the GNU General Public License as published by  *
 * the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                   *
 *                                                                       *
 * This program is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 * GNU General Public License for more details.                          *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *************************************************************************/

#include "twi_master.h"
#include <util/delay.h>

#ifdef TWI_DEBUG
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "avr-uart/uart.h"
//void twi_master_print_status(void);
#endif

static uint8_t twst;

/*******************************************************************************
Call this function to init the TWI hardware.
*******************************************************************************/
void twi_master_init()
{
  TWSR = TWI_TWPS;
  TWBR = TWI_TWBR;
#ifdef TWI_DEBUG
  char buf[10];
  uart_puts_P("\ntwi_master_init: F_SCL=");
  ltoa(TWI_SCL_CLOCK, buf, 10);
  uart_puts(buf);
  uart_puts_P(" Hz, TWBR=");
  utoa(TWI_TWBR, buf, 10);
  uart_puts(buf);
  uart_putc('\n');
  //printf("\ntwi_master_init: F_SCL=%ld Hz, TWBR=%ld\n", TWI_SCL_CLOCK,
  //  TWI_TWBR);
#endif
}

/*******************************************************************************
Try to transmit the START condition.

RETURN VALUE
  If no error occurs, this function returns the TWI Status Register value.
  On error, 0 is returned.
*******************************************************************************/
uint8_t twi_master_start()
{
  uint16_t timeout = 1;
#ifdef TWI_DEBUG
  char buf[10];
  uart_puts_P("\ntwi_start: ");
  //printf_P(PSTR("\ntwi_start:\t\t"));
#endif
  // send START condition
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  // TODO fully interrupt driven routines
  // wait until transmission is completed
  while (!(TWCR & _BV(TWINT)) && timeout)
    timeout++;
  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS;
#ifdef TWI_DEBUG
  //twi_master_print_status();
#endif
  if (timeout == 0)
#ifdef TWI_DEBUG
  {
      uart_puts_P(", TIMEOUT\n");
    //printf_P(PSTR(",\t\tTIMEOUT\n"));
#endif
    return 0;
#ifdef TWI_DEBUG
  }
  else
  {
    uart_puts_P(", cycles: ");
    utoa(timeout, buf, 10);
    uart_puts(buf);
    uart_putc('\n');
    //printf(",\t\tcycles: %2d\n", timeout);
  }
#endif

  return ((twst == TW_START) || (twst == TW_REP_START)) ? twst : 0;
}

/*******************************************************************************
Transmit STOP condition and reset TWI interface.
*******************************************************************************/
void twi_master_stop()
{
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
#ifdef TWI_DEBUG
  uart_puts_P("twi_stop\n");
  //printf_P(PSTR("twi_stop\n"));
#endif
}

/*******************************************************************************
Try to transmit one data byte.

ARGUMENTS
  1-st : data byte to be transmitted

RETURN VALUE
  If no error occurs, this function returns the TWI Status Register value.
  On error, 0 is returned.
*******************************************************************************/
uint8_t twi_master_write(const uint8_t data)
{
  uint16_t timeout = 1;
#ifdef TWI_DEBUG
  char buf[10];
  uart_puts_P("wi_write[");
  utoa(data, buf, 10);
  uart_puts(buf);
  uart_puts_P("]: ");
  //printf("twi_write[0x%2X]:\t", data);
#endif
  // Set TWI registers and start transmission of data
  TWDR = data;
  TWCR = _BV(TWINT) | _BV(TWEN);
  // wail until transmission is completed and ACK/NACK has been received
  while (!(TWCR & _BV(TWINT)) && timeout)
    timeout++;
  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS;
#ifdef TWI_DEBUG
  //twi_master_print_status();
#endif
  if (timeout == 0)
#ifdef TWI_DEBUG
  {
    uart_puts_P(", TIMEOUT\n");
    //printf_P(PSTR(",\t\tTIMEOUT\n"));
#endif
    return 0;
#ifdef TWI_DEBUG
  }
  else
  {
    uart_puts_P(", cycles: ");
    utoa(timeout, buf, 10);
    uart_puts(buf);
    uart_putc('\n');
    //printf(",\t\tcycles: %d\n", timeout);
  }
#endif

  return (twst == TW_MT_SLA_ACK) || (twst == TW_MT_DATA_ACK) ||
         (twst == TW_MR_SLA_ACK) ? twst : 0;
}

/*******************************************************************************
Read one data byte from slave and send ACK or NAK in return.

ARGUMENTS
  1-st : pointer to the buffer for received data
  2-nd : either TWI_NAK or TWI_ACK
  
RETURN VALUE
  If no error occurs, this function returns the TWI Status Register value.
  On error, 0 is returned.
*******************************************************************************/
uint8_t twi_master_read(uint8_t *buf, const uint8_t ack)
{
  uint16_t timeout = 1;
#ifdef TWI_DEBUG
  char tmp_buf[10];
#endif

  // Set TWI registers
  if (ack)
  {
#ifdef TWI_DEBUG
    uart_puts_P("twi_read[TWI_ACK, ");
    //printf_P(PSTR("twi_read[TWI_ACK,"));
#endif
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
  }
  else
  {
#ifdef TWI_DEBUG
    uart_puts_P("twi_read[TWI_NAK, ");
    //printf_P(PSTR("twi_read[TWI_NAK,"));
#endif
    TWCR = _BV(TWINT) | _BV(TWEN);
  }
  // wait until reading completed
  while (!(TWCR & _BV(TWINT)) && timeout)
    timeout++;
  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS;
#ifdef TWI_DEBUG
  utoa(TWDR, tmp_buf, 10);
  uart_puts(tmp_buf);
  uart_puts_P("]: ");
  //printf("0x%2X]:\t", TWDR);
  //twi_master_print_status();
#endif
  if (timeout == 0)
  {
#ifdef TWI_DEBUG
    uart_puts_P(", TIMEOUT\n");
    //printf_P(PSTR(",\tTIMEOUT\n"));
#endif
    *buf = 0;
    return 0;
  }
  else
  {
    *buf = TWDR;
#ifdef TWI_DEBUG
    uart_puts_P(", cycles: ");
    utoa(timeout, tmp_buf, 10);
    uart_puts(tmp_buf);
    uart_putc('\n');
    //printf(",\tcycles: %d\n", timeout);
#endif
  }

  return (twst == TW_MR_DATA_ACK) || (twst == TW_MR_DATA_NACK) ? twst : 0;
}

/*******************************************************************************
Write one byte to the 8-bit register of the specified slave.

ARGUMENTS
  1-st : slave address (SAD)
  2-nd : target register address
  3-rd : data byte
  
RETURN VALUE
  If no error occurs, this function returns positive value, 0 otherwise.
*******************************************************************************/


/*******************************************************************************
Write one byte to the 8-bit register of the specified slave.

ARGUMENTS
  1-st : slave address (SAD)
  2-nd : target register address 16 bit
  3-rd : data byte
  
RETURN VALUE
  If no error occurs, this function returns positive value, 0 otherwise.
*******************************************************************************/

/******************************************************************************
Write page to memory
*******************************************************************************/
uint8_t twi_master_write_page(const uint8_t sad, const uint16_t reg_addr,
                              const uint8_t length, uint16_t data[])
{
  uint8_t st = 1;
  uint8_t i=4;
  
  //uint16_t tst=34831;
  
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  
  st = st && twi_master_write(reg_addr >> 8);
  st = st && twi_master_write(reg_addr);
  
  for (i; i < length+4; i++) //!!!!!!!!!!!!!!!
  {
        //st = st && twi_master_write((uint8_t)(data[i]));
    //st = st && twi_master_write((uint8_t)(data[i])>>8);
    st = st && twi_master_write((uint8_t)(data[i]));
  }
  twi_master_stop();

  return st;
}

uint8_t twi_master_read16_reg8(const uint8_t sad, const uint16_t reg_addr)
{
  uint8_t st = 1;
  uint8_t buf = 0b00000000;
  uint8_t value = 0;
  
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  
  st = st && twi_master_write(reg_addr >> 8);
  st = st && twi_master_write(reg_addr);
  
  //twi_master_stop();
  //_delay_ms(1);
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_READ);
  
  st = st && twi_master_read(&value, TWI_NAK);
  
  twi_master_stop();

  return value;
}

uint16_t twi_master_read16_reg16(const uint8_t sad, const uint16_t reg_addr)
{
    uint16_t buf = 0b0000000000000000;
    uint8_t value = 0;
  uint8_t st = 1;

  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  
  st = st && twi_master_write(reg_addr >> 8);
  st = st && twi_master_write(reg_addr);
  
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_READ);
  
    st = st && twi_master_read(&value, TWI_ACK);
    buf = (value << 8);
    
    st = st && twi_master_read(&value, TWI_NAK);
    buf |= value;
    
  twi_master_stop();

  return buf;
}

uint8_t twi_master_write16_reg8(const uint8_t sad, const uint16_t reg_addr,
                              uint8_t data)
{
  uint8_t st = 1;
  
  st = st && twi_master_start();
  
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  
  st = st && twi_master_write(reg_addr >> 8);
  st = st && twi_master_write(reg_addr);
  
  st = st && twi_master_write(data);
  
  twi_master_stop();
  _delay_ms(10);
  return st;
}

uint8_t twi_master_write16_reg16(const uint8_t sad, const uint16_t reg_addr,
                              uint16_t data)
{
  uint8_t st = 1;
  
  st = st && twi_master_start();
  
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  
  st = st && twi_master_write(reg_addr >> 8);
  st = st && twi_master_write(reg_addr);
  
  st = st && twi_master_write(data >> 8);
  st = st && twi_master_write(data);
  
  twi_master_stop();
  _delay_ms(10);
  return st;
}

/*******************************************************************************
Write two bytes (one word) to the 16-bit register of the specified slave.

ARGUMENTS
  1-st : slave address (SAD)
  2-nd : target register address
  3-rd : data word

RETURN VALUE
  If no error occurs, this function returns positive value, 0 otherwise.
*******************************************************************************/

uint8_t twi_master_write_reg8(const uint8_t sad, const uint8_t reg_addr,
                              const uint8_t data)
{
  uint8_t st = 1;
  
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  st = st && twi_master_write(reg_addr);
  st = st && twi_master_write(data);
  twi_master_stop();

  return st;
}

uint8_t twi_master_write_reg16(const uint8_t sad, const uint8_t reg_addr,
                               const uint16_t data)
{
  uint8_t st = 1;

  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  st = st && twi_master_write(reg_addr);
  st = st && twi_master_write(data >> 8); //st = st && twi_master_write((uint8_t) data >> 8);
  st = st && twi_master_write(data & 0xFF); //st = st && twi_master_write((uint8_t) data & 0xFF);
  twi_master_stop();

  return st;
}

/*******************************************************************************
Read one byte from the 8-bit register of the specified slave.

ARGUMENTS
  1-st : slave address (SAD)
  2-nd : target register address
  3-rd : pointer to the buffer for received data

RETURN VALUE
  If no error occurs, this function returns positive value, 0 otherwise.
*******************************************************************************/
uint8_t twi_master_read_reg8(const uint8_t sad, const uint8_t reg_addr,
                             uint8_t *buf)
{
  uint8_t st = 1;

  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE);
  st = st && twi_master_write(reg_addr);
  //twi_master_stop();
  //_delay_ms(1);
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_READ);
  st = st && twi_master_read(buf, TWI_NAK);
  twi_master_stop();

  return st;
}

/*******************************************************************************
Read two bytes (one word) from the 16-bit register of the specified slave.

ARGUMENTS
  1-st : slave address (SAD)
  2-nd : target register address
  3-rd : pointer to the buffer for received data

RETURN VALUE
  If no error occurs, this function returns positive value, 0 otherwise.
*******************************************************************************/
uint8_t twi_master_read_reg16(const uint8_t sad, const uint8_t reg_addr,
                              int16_t *buf, const uint8_t byte_order)
{
  uint8_t value = 0;
  uint8_t st = 1;

  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_WRITE); //st = st && twi_master_write((sad << 1) | TW_WRITE);
  st = st && twi_master_write(reg_addr);
  
  //twi_master_stop();
  //_delay_ms(1);
  
  st = st && twi_master_start();
  st = st && twi_master_write((sad << 1) | TW_READ);
  
  st = st && twi_master_read(&value, TWI_ACK);

  if (byte_order == TWI_MSB_FIRST)
    *buf = (value << 8);
  else // byte_order == TWI_LSB_FIRST
    *buf = value;

  st = st && twi_master_read(&value, TWI_NAK);
  twi_master_stop();

  if (byte_order == TWI_LSB_FIRST)
    *buf |= value;
  else // byte_order == TWI_LSB_FIRST
    *buf |= (value << 8);
    

  return st;
}

/*******************************************************************************
Print debug message according to the value in twst.
*******************************************************************************/
#ifdef TWI_DEBUG1
void twi_master_print_status()
{
  char buf[10];
    
  switch (twst)
  {
    case TW_START:
      uart_puts_P("TW_START");
    break;

    case TW_REP_START:
      uart_puts_P("TW_REP_START");
    break;

    case TW_MT_SLA_ACK:
      uart_puts_P("TW_MT_SLA_ACK");
    break;

    case TW_MT_SLA_NACK:
      uart_puts_P("TW_MT_SLA_NACK");
    break;

    case TW_MT_DATA_ACK:
      uart_puts_P("TW_MT_DATA_ACK");
    break;

    case TW_MT_DATA_NACK:
      uart_puts_P("TW_MT_DATA_NACK");
    break;

    case TW_MT_ARB_LOST:
      uart_puts_P("TW_MT_ARB_LOST");
    break;

    case TW_MR_SLA_ACK:
      uart_puts_P("TW_MR_SLA_ACK");
    break;

    case TW_MR_SLA_NACK:
      uart_puts_P("TW_MR_SLA_NACK");
    break;

    case TW_MR_DATA_ACK:
      uart_puts_P("TW_MR_DATA_ACK");
    break;

    case TW_MR_DATA_NACK:
      uart_puts_P("TW_MR_DATA_NACK");
    break;

    case TW_NO_INFO:
      uart_puts_P("TW_NO_INFO");
    break;

    case TW_BUS_ERROR:
      uart_puts_P("TW_BUS_ERROR");
    break;

    default:
      utoa(twst, buf, 10);
      uart_puts(buf);
      //printf("0x%2X", twst);
    break;
  }
}
#endif // TWI_DEBUG
