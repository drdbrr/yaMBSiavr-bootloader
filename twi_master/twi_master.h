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

//
// These constants should be defined in project's main.h
//
// TWI_SCL_CLOCK (in Hz)
// TWI_DEBUG (1|0)
//

#ifndef _TWI_MASTER_H
#define _TWI_MASTER_H

//
// This library should be located in 'twi_master' subdir of your project
//
#include <inttypes.h>
#include <util/twi.h>

// F_SCL = F_CPU/(16+2*(TWBR)*4^TWPS)
// TWBR - TWI bit rate register
// TWPS - prescaler TWI status register
#define TWI_SCL_CLOCK 80000 
#define  TWI_TWBR ((F_CPU / TWI_SCL_CLOCK) - 16) / 2
//#define  TWI_TWBR 0x0C
#define  TWI_TWPS 0

// TWI defines
#define  TWI_NAK 0
#define  TWI_ACK 1

// Byte order for twi_master_read_reg16()
#define  TWI_MSB_FIRST 0
#define  TWI_LSB_FIRST 1

//#define TW_READ 0x01
//#define TW_WRITE 0x00

void twi_master_init(void);
uint8_t twi_master_start(void);
void twi_master_stop(void);
uint8_t twi_master_write(const uint8_t);
uint8_t twi_master_read(uint8_t *, const uint8_t);

//***********************EEPROM
uint8_t twi_master_write_page(const uint8_t sad, const uint16_t reg_addr,
                              const uint8_t length,uint16_t data[]);

uint8_t twi_master_read16_reg8(const uint8_t, const uint16_t);
uint16_t twi_master_read16_reg16(const uint8_t sad, const uint16_t reg_addr);

uint8_t twi_master_write16_reg8(const uint8_t sad, const uint16_t reg_addr, uint8_t data);
uint8_t twi_master_write16_reg16(const uint8_t sad, const uint16_t reg_addr, uint16_t data);
//***********************

uint8_t twi_master_write_reg8(const uint8_t, const uint8_t, const uint8_t);
uint8_t twi_master_write_reg16(const uint8_t, const uint8_t, const uint16_t);
uint8_t twi_master_read_reg8(const uint8_t, const uint8_t, uint8_t *);
uint8_t twi_master_read_reg16(const uint8_t, const uint8_t, int16_t *,
                              const uint8_t );

#endif //_TWI_MASTER_H
