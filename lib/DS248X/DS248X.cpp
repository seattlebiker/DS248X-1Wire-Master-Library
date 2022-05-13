/******************************************************************* 
 * A library for interfacing an ESP8266 to DS2482-100, DS2482-800, 
 * and DS2484 I2C-to-1-wire masters. (should work also for Arduino's)
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 * See <http://www.gnu.org/licenses/>.
 * 
 * All files, software, schematics and designs are for experimental
 * and/or hobby use.
 * 
 * Under no circumstances should any part be used for critical systems 
 * where safety, life or property depends upon it. You are responsible 
 * for all use.
 *   You are free to use, modify, derive or otherwise extend for your 
 *   own non-commercial purposes provided
 *       1. No part of this software or design may be used to cause 
 * 	  	injury or death to humans or animals.
 *       3. Credit is given to the author(s), and links are provided to the 
 * 	  	original source.
 * 
 * Thanks to various authors' code using 1-wire devices to refresh my
 * brain, even though I've been using 1-wire for years.  Interfacing an ESP8266
 * with a DS2482 1-wire master has been a learning experience, and I acknowledge
 * the work below.  I've incorporated ideas from many sources for my 
 * DS248X 1-wire library, and adapted them in my code.  I changed some of the
 * function names where it makes sense, i.e., diferentiating the DS2482 device
 * reset from a 1-wire reset, e.g. OWReset.
 * 
 * Paul Stoffregan maintains the 1-wire library (originally by Jim Studt).  The 
 * latest version of which may be found at:
 *		http://www.pjrc.com/teensy/td_libs_OneWire.html
 * 
 * I also got some ideas from these authors:
 * 
 * PaeaeTech: https://github.com/paeaetech/paeae/tree/master/Libraries/ds2482
 * 
 * CyberGibbons: https://github.com/cybergibbons/DS2482_OneWire
 * 
 * Cybergibbons' library, is cool, but it's based on Paeae's library which
 * has the same issue as follows:
 * 
 * I had to fix the 1-wire search function because it failed to find other devices 
 * on the MicroLan if a device with bit 64 in the ROM was a '1' was also on the bus.
 * I don't take credit for that search fix, but it's in my version. Here's a couple
 * links; User "Garitron" on the Arduino forum found the fix for the 1-wire search 
 * detailed in the first link.
 *  
 *   https://forum.arduino.cc/t/introducing-maxim-ds2482-1-wire-master-library-2/51436
 *   https://forum.arduino.cc/t/introducing-maxim-ds2482-1-wire-master-library/37430?utm_source=pocket_mylist
 * 
 * As a side note, I'm including the following from the DS2482-100/800 Specifications 
 * PDF since I think it's helpful, instead of having to always refer to to the APP note.  
 * This has helped me, anyway.  :-)
 * 
 * =======================================================================
 * 
 * The DS2482 understands eight function commands, which fall into four 
 * categories: device control, I²C communication, 1-Wire set-up and 
 * 1-Wire communication. The feedback path to the host is controlled by a read
 * pointer, which is set automatically by each function command for the 
 * host to efficiently access relevant information.
 * The host processor sends these commands and applicable parameters as 
 * strings of one or two bytes using the I²C interface. The I²C protocol 
 * requires that each byte be acknowledged by the receiving party to confirm 
 * acceptance or not be acknowledged to indicate an error condition 
 * (invalid code or parameter) or to end the communication.
 * 
 * DEVICE REGISTERS
 * The DS2482 has three registers that the I²C host can read: Configuration, 
 * Status, and Read Data. These registers are addressed by a read pointer. 
 * The position of the read pointer, i.e., the register that the host reads in a
 * subsequent read access, is defined by the instruction that the has DS2482 
 * executed last. The host has read and write access to the Configuration register 
 * to enable certain 1-Wire features.
 * Configuration Register
 * The DS2482 supports allows four 1-Wire features that are enabled or selected 
 * through the Configuration register.
 * These features are:
 *     Active Pullup (APU)
 *     Presence Pulse Masking (PPM)
 *     Strong Pullup (SPU)
 *     1-Wire Speed (1WS)
 * 
 * These features can be selected in any combination. While APU, PPM, and 
 * 1WS maintain their state, SPU returns to its inactive state as soon 
 * as the strong pullup has ended.
 * Active Pullup (APU) 
 * The APU bit controls whether an active pullup (controlled slew-rate 
 * transistor) or a passive pullup (RWPU resistor) is used to drive a 
 * 1-Wire line from low to high. When APU = 0, active pullup is disabled 
 * (resistor mode). Active Pullup should be selected if the 1-Wire line 
 * has a substantial length (several 10m) or if there is a large number 
 * (~20 or more) of devices connected to a 1-Wire line. The active pullup 
 * does not apply to the rising edge of a presence pulse or a recovery 
 * after a short on the 1-Wire line. The circuit that controls rising edges 
 * (Figure 2 of DS2482 PDF) operates as follows: At t1 the pulldown 
 * (from DS2482 or 1-Wire slave) ends. From this point on the 1-Wire bus 
 * is pulled high through RWPU internal to the DS2482. VCC and the capacitive 
 * load of the 1-Wire line determine the slope.  In case that active pullup 
 * is disabled (APU = 0), the resistive pullup continues, as represented 
 * by the solid line. With active pullup enabled (APU = 1), when at t2 
 * the voltage has reached a level between VIL1max and VIH1min, the DS2482 
 * actively pulls the 1-Wire line high applying a controlled slew rate, 
 * as represented by the dashed line. The active pullup continues until 
 * tAPUOT is expired at t3. From that time on the resistive pullup will 
 * continue.
 * 
 *******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "DS248X.h"

uint8_t getCRC8( uint8_t *addr, uint8_t len);
uint16_t getCRC16( uint8_t *data, uint8_t len);

byte RESET_NOPRESENCE 	= 0x00;	// MicroLan reset result = no presence
byte RESET_PRESENCE 	= 0x01;	// MicroLan reset result = presence
byte RESET_ALARM 		= 0x02;	// MicroLan reset result = ALARM presence
byte RESET_SHORT 		= 0x03;	// MicroLan reset result = Shorted

void printArrayInHex (uint8_t array[], int len)
{
  Serial.print("{ ");
  for (uint8_t i = 0; i < len; i++)
  {
    // zero pad the address if necessary
    Serial.print("0x");
    if (array[i] < 16) Serial.print("0");
    Serial.print(array[i], HEX);
    if ( !(i % 8) ) Serial.print("");
    if (i < len - 1) Serial.print(", ");    
  }
  Serial.print(" }");
}

// Constructor with no parameters for compatability with DS248X lib
DS248X::DS248X()
{
	// Address is determined by two pins on the DS2482 AD1/AD0
	// Pass 0b00, 0b01, 0b10 or 0b11
	mAddress = 0x18;
	mError = 0;
}

DS248X::DS248X(uint8_t address)
{
	// Address is determined by two pins on the DS2482 AD1/AD0
	// Pass 0b00, 0b01, 0b10 or 0b11
	mAddress = 0x18 | address;
	mError = 0;
}

uint8_t DS248X::getAddress()
{
	return mAddress;
}

uint8_t DS248X::getError()
{
	return mError;
}

// Helper functions to make dealing with I2C side easier
void DS248X::begin()
{
	Wire.beginTransmission(mAddress);
}

uint8_t DS248X::end()
{
	return Wire.endTransmission();
}

void DS248X::I2CwriteByte(uint8_t data)
{
	Wire.write(data); 
}

uint8_t DS248X::I2CreadByte()
{
	Wire.requestFrom(mAddress,1u);
	return Wire.read();
}

// Simply starts and ends an Wire transmission
// If no devices are present, this returns false
uint8_t DS248X::isConnected()
{
	begin();
	return !end() ? true : false;
}

/*************************************************************************
 * Resets the DS2482 1-wire bridge which does a global reset of the device 
 * state-machine logic and terminates any ongoing 1-Wire communication.
 * The command code for the device reset is 0xF0.
 * 
 * @return void
 * 
 *************************************************************************/
void DS248X::DS2482Reset()
{
	begin();
	I2CwriteByte(DS248X_COMMAND_RESET);
	end();
}

/*************************************************************************
 * Sets the read pointer to the specified register To prepare reading the 
 * result from a 1-Wire Byte command; random read access of registers.  
 * Overwrites the read pointer position of any 1-Wire communication command 
 * in progress.
 * Register Selection Codes: Status Register=F0h, Read Data Register=E1h,
 *	 Channel Selection Register=D2h, Configuration Register=C3h 
 * @param reg to one of the above selected registers
 * @returns void
 *************************************************************************/
void DS248X::setReadPointer(uint8_t reg)
{
	begin();
	I2CwriteByte(DS248X_COMMAND_SRP);
	I2CwriteByte(reg);
	end();
}

/*****************************************************************
 * Read the DS2482 status register
 * All 1-Wire communication commands and the Device Reset command 
 * position the read pointer at the Status register for the host 
 * processor to read with minimal protocol overhead. Status information 
 * is updated during the execution of certain commands only. Details 
 * are given in the description of the various status bits below.
 *
 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 *========================================
 *  DIR  TSB  SBR  RST   LL   SD  PPD  1WB
 *
 ******************************************************************/ 
uint8_t DS248X::readStatus()
{
	setReadPointer(DS248X_POINTER_STATUS);
	return I2CreadByte();
}

/*************************************************************************
 * Sets the read pointer to the specified register. Overwrites 
 * the read pointer position of any 1-Wire communication command in progress.
 * 
 * @param readPointer to beginning of that memory for reading
 * @returns void
 *************************************************************************/
uint8_t DS248X::readData()
{
	setReadPointer(DS248X_POINTER_DATA);
	return I2CreadByte();
}

// After a device reset (power-up cycle or initiated by the Device Reset command) 
// the Configuration register reads 00h. When writing to the Configuration register, 
// the new data is accepted only if the upper nibble (bits 7 to 4) is the one's complement 
// of the lower nibble (bits 3 to 0). When read, the upper nibble is always 0h.
//
// bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
//  ___  ___  ___  ___
//  1WS  SPU  PPM  APU  1WS  SPU  PPM  APU
// Read the config register
uint8_t DS248X::readConfig()
{
	setReadPointer(DS248X_POINTER_CONFIG);
	return I2CreadByte();
}

// Strong Pullup (SPU) 
// The SPU bit controls whether the DS2482 applies a low-impedance 
// pullup to VCC on the 1-Wire line after the last bit of either a 1-Wire 
// Write Byte command or after a 1-Wire Single Bit command has completed. 
// The strong pullup feature is commonly used with 1-Wire EEPROM devices 
// when copying scratchpad data to the main memory or when performing a 
// SHA-1 computation, and with parasitically powered temperature sensors or 
// A-to-D converters. The respective device data sheets specify the 
// location in the communications protocol after which the strong pullup 
// should be applied. The SPU bit in the configuration register of the DS2482
// must be set immediately prior to issuing the command that puts the 1-Wire 
// device into the state where it needs the extra power. If SPU is 1, the 
// DS2482 applies active pullup to the rising edge of the time slot in which 
// the strong pullup starts, regardless of the APU bit setting. However, 
// in contrast to setting APU = 1 for active pullup, the low-impedance 
// pullup will not end after tAPUOT is expired. Instead, as shown in 
// Figure 4, the low-impedance pullup remains active  until: 
//     a) the next 1-Wire communication command (the typical case), 
//     b) by writing to the Configuration Register with the SPU bit being 0 
//        (alternative), or 
//     c) by issuing the Device Reset command. 
// Additionally, when the pullup ends, the SPU bit is automatically reset to 0. 
// Using the strong pullup does not change the state of the APU bit in the 
// Configuration Register.
//
void DS248X::setStrongPullup()
{
	writeConfig(readConfig() | DS248X_CONFIG_SPU);
}

void DS248X::clearStrongPullup()
{
	writeConfig(readConfig() & !DS248X_CONFIG_SPU);
}

// Churn until the busy bit in the status register is clear
uint8_t DS248X::waitOnBusy()
{
	uint8_t status;

	for(int i=1000; i>0; i--)
	{
		status = readStatus();
		if (!(status & DS248X_STATUS_BUSY))
			break;
		delayMicroseconds(20);		
	}

	// if we have reached this point and we are still busy, there is an error
	if (status & DS248X_STATUS_BUSY)
		mError = DS248X_ERROR_TIMEOUT;

	// Return the status so we don't need to explicitly do it again
	return status;
}


/*******************************************************************
 * Writes a new configuration byte. The new settings take effect 
 * immediately. When writing to the Configuration register,  the new 
 * data is accepted only if the upper nibble (bits 7 to 4) is the one's 
 * complement of the lower nibble (bits 3 to 0). When read, the upper 
 * nibble is always 0h.
 * @returns void
 ********************************************************************/
void DS248X::writeConfig(uint8_t config)
{
	waitOnBusy();
	begin();
	I2CwriteByte(DS248X_COMMAND_WRITECONFIG);
	// Write the 4 bits and the complement 4 bits
	I2CwriteByte(config | (~config)<<4);
	end();
	
	// This should return the config bits without the complement
	if (I2CreadByte() != config)
		mError = DS248X_ERROR_CONFIG;
}

/*********************************************************************
 *  1-Wire reset seatch algorithm
 * @returns void
 ********************************************************************/
void DS248X::OWResetSearch()
{
	searchLastDiscrepancy = 0;
	searchLastDeviceFlag = false;

	for (int i = 0; i < 8; i++)
	{
		ROM_ID[i] = 0;
	}
}


/*********************************************************************
 * OWReset()
 * Generates a 1-Wire reset/presence-detect cycle on the 1-Wire line. 
 * The state of the 1-Wire line is reported to the host processor 
 * through the  Status Register, bits PPD and SD.  1-Wire activity must 
 * have ended before the DS2482 can process this command. Strong pullup 
 * (see SPU bit) should not be used in conjunction with the 1-Wire Reset 
 * command. If SPU is enabled, the PPD bit may not be valid and may cause 
 * a violation of the device's absolute maximum rating. 
 * 
 * @returns 1 if there was a presence pulse detected, 0 if not
 *   bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 *   ========================================
 *    DIR  TSB  SBR  RST   LL   SD  PPD  1WB
 * 
 *******************************************************************/
uint8_t DS248X::OWReset()
{
	waitOnBusy();
	clearStrongPullup();		// Clear the SPU bit just in case
	begin();
	I2CwriteByte(DS248X_COMMAND_RESETWIRE);
	end();

	uint8_t status = waitOnBusy();
	if (status & DS248X_STATUS_SD)
	{
		mError = DS248X_ERROR_SHORT;
	}	
	return (status & DS248X_STATUS_PPD) ? true : false;
}


/**
 * (Original-OWreset)
 * Does a reset of the 1-wire bus via the DS2482 I2C 1-wire bridge
 * @return status byte from DS2482
 * <p>
 *  bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
 *  ========================================
 *  DIR  TSB  SBR  RST   LL   SD  PPD  1WB
 * 
 */
//int OWReset() { 
//	void I2CwriteByte(uint8_t data);
//	uint8_t I2CreadByte();
//
//    byte reset_cmd = DS2482_1WireResetCmd;
//    int poll_count = 0;
//    byte status_reg = 0;
//    I2CwriteByte(reset_cmd);
//    do {           
//        status_reg = I2CreadByte();
//        poll_count++;
//    } while (0x01 == (status_reg & DS248X_STATUS_BUSY) && poll_count < POLL_LIMIT);
//    
//    if((status_reg & DS248X_STATUS_PPD) == DS248X_STATUS_PPD)  {
//        return RESET_PRESENCE;
//    } else if ((status_reg & DS248X_STATUS_SD) == DS248X_STATUS_SD ) {
//        return RESET_SHORT;
//    } else {
//        return RESET_NOPRESENCE;
//    }
//}

/*********************************************************************
 * Writes a single data byte to the 1-Wire line.
 * @param data  Byte to be written
 * @param power 1 = turn on Strong pullup (SPU), 0 = No Strong pullup
 * @returns void
 *********************************************************************/
void DS248X::OWWriteByte(uint8_t data, uint8_t power)
{
	waitOnBusy();
	if (power)
		setStrongPullup();
	begin();
	I2CwriteByte(DS248X_COMMAND_WRITEBYTE);
	I2CwriteByte(data);
	end();
}

/*********************************************************************
 * Generates eight read-data time slots on the 1-Wire line and stores 
 * result in the Read Data Register.
 * @returns byte read from 1-wire device
 *********************************************************************/
uint8_t DS248X::OWReadByte()
{
	waitOnBusy();	// Definitely needed here
	begin();
	I2CwriteByte(DS248X_COMMAND_READBYTE);
	end();
	waitOnBusy();	// Definitely needed here
	return readData();
}

/*********************************************************************
 * Generates a single 1-Wire time slot with a bit value “V” as specified 
 * by the bit byte at the 1-Wire line. A V value of 0b generates a write-zero 
 * time slot; a V value of 1b generates a write-one time slot, which also 
 * functions as a read-data time slot. In either case, the logic level at 
 * the 1-Wire line is tested at tMSR and SBR is updated.
 * 
 * Bit Allocation in the Bit Byte 
 *  bit 7 bit 6 bit 5 bit 4 bit 3 bit 2 bit 1 bit 0 
 *    V    x    x     x     x      x     x     x 
 * 
 * @param data  Bit value to be written (bit 7 of the data byte)
 * @param power 1 = turn on Strong pullup (SPU), 0 = No Strong pullup
 * @returns void
 *********************************************************************/
void DS248X::OWWriteBit(uint8_t data, uint8_t power)
{
	//waitOnBusy();
	if (power)
		setStrongPullup();
	begin();
	I2CwriteByte(DS248X_COMMAND_SINGLEBIT);
	I2CwriteByte(data ? 0x80 : 0x00);
	end();
}

// As OWWriteBit
uint8_t DS248X::OWReadBit()
{
	OWWriteBit(1);
	uint8_t status = waitOnBusy();
	return status & DS248X_STATUS_SBR ? 1 : 0;
}

/*********************************************************************
 * This command can save time in a single drop bus system by allowing 
 * the bus master to access the memory functions without providing the 
 * 64–bit ROM code. If more than one slave is present on the bus and a read
 * command is issued following the Skip ROM command, data collision will 
 * occur on the bus as multiple slaves will transmit simultaneously (open 
 * drain pulldowns will produce a wired AND result)
 * @returns void
 *********************************************************************/
void DS248X::OWSkipROM()
{
	OWWriteByte(OWSkipROMCmd, 0);		// OWSkipROM() [CCh]
}

/*********************************************************************
 * OWSelect(uint8_t rom[8])
 *
 * Selects the specified iButton or 1-Wire device by broadcasting its
 * address.  This operation is referred to a 'MATCH ROM' operation
 *  in the 1-Wire device data sheets.  This does not affect the 'current' 
 * device state information used in searches (findNextDevice...).
 * 
 * Warning, this does not verify that the device is currently present
 * on the 1-Wire Network (See isPresent). 
 * @param ROM ID (64 bits)
 * @returns void
 **********************************************************************/
void DS248X::OWSelect(uint8_t rom[8])
{
	OWWriteByte(OWMatchROMCmd, 0);
	for (int i=0;i<8;i++)
		OWWriteByte(rom[i], 0);
}


/*********************************************************************
 * Select the 1-Wire channel on a DS2482-800.
 * @param ch Channel to be used on the DS2482
 * @returns TRUE if channel selected, FALSE device not detected 
 * or failure to perform select
 *********************************************************************/
uint8_t DS248X::selectChannel(byte ch) {
  uint8_t w[] = {0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5, 0x96, 0x87};
  uint8_t r[] = {0xb8, 0xb1, 0xaa, 0xa3, 0x9c, 0x95, 0x8e, 0x87};
  //waitOnBusy();
  begin();
  I2CwriteByte(0xc3);
  I2CwriteByte(w[ch]);
  end();
  // Not looking like the below wait is needed...
  //waitOnBusy();
  return I2CreadByte() == r[ch];
}

/*********************************************************************
 * The 1-Wire CRC scheme is described in Maxim Application Note 27:
 * "Understanding and Using Cyclic Redundancy Checks with Maxim iButton 
 * Products".  Also:
 * https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812
 * @param addr Pointer to array of bytes for CRC calculation
 * @param len number of bytes
 * @returns 8 bit Cyclic Redundancy Check byte
 *********************************************************************/
uint8_t getCRC8( uint8_t *addr, uint8_t len)
{
    uint8_t crc=0;   
    for (uint8_t i=0; i<len;i++) 
    {
        uint8_t inbyte = addr[i];
        for (uint8_t j=0;j<8;j++) 
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) 
                  crc ^= 0x8C;
            
            inbyte >>= 1;
        }
    }
    return crc;
}

/* Setup the search to find the device with 'family_code' on the next call
 to OWSearch. family_code if it is present on the Micronlan.
 (Copied from the OneWire library with minor changes.)

The 'TARGET SETUP' operation is a way to preset the search state to first 
find a particular family type.  Each 1-Wire device has a one byte family 
code embedded within the ROM identification number. This family code allows 
the 1-Wire master to know what operations this device is capable of. If there are
multiple devices on the 1-Wire it is common practice to target a search to only 
the family of devices that are of interest. To target a particular family, 
set the desired family code byte into the first byte of the ROM_ID array 
and fill the rest of the ROM_ID array with zeros. Then set the LastDiscrepancy 
to 64 (40 hex) and both LastDeviceFlag and LastFamilyDiscrepancy to 0. When 
OWSearch is next performed, the FIRST device of the desired family 
type is discovered and placed in the ROM_ID array. Note that if no devices 
of the desired family are currently on the 1-Wire, then another type will be found, 
so the family code in the resulting ROM_NO must be verified after the search.
@param family_code 1-byte family code in hex
@returns Void
*/
void DS248X::OWTargetSearch(uint8_t family_code)
{
   	// set the search state to find SearchFamily type devices
	ROM_ID[0] = family_code;
   	for (uint8_t i = 1; i < 8; i++)
      	ROM_ID[i] = 0;
	
	searchLastDiscrepancy = 64;
	searchLastFamilyDiscrepancy = 0;
	searchLastDeviceFlag = false;
}


/**
  * OWSearch does a 1-wire search using the DS2482 1-wire Triplet command.
  * 
  * This code was taken from the Dallas/Maxim Application note AN3684 "How
  * to Use the DS2482 1-wire Master" and orinially modified for Java ME, and
  * it also works great for an Espressif/Arduino-connected DS2482.  It's modified 
  * somewhat in that the found device isn't put in the global "ROM_NO" buffer,
  * but rather copied to the array buffer pointed to by dev_addr.
  * 
  * Also see the Maxim/Dallas PDF for DS2482-100 and DS2482-800 devices.
  * 
  * searchLastDeviceFlag - If the last device has been found (true), or not (false)
  * dev_addr - the returned serial number
  * 
  * When 'doAlarmSearch' is TRUE (1) the find alarm command
  * 0xEC is sent instead of the normal search command 0xF0.
  * Using the find alarm command 0xEC will limit the search to only
  * 1-Wire devices that are in an 'alarm' state.
  * 
  * @returns TRUE if a 1-wire device was found and its ROM ID is
  * placed in dev_addr, or FALSE if no new device was found.
  * @param searchMode If searchMode is true, then a normal search is done.
  * Otherwise, if false, then an ALARM SEARCH is started.  
  */   
boolean DS248X::OWSearch(uint8_t *dev_addr, bool searchMode)
{
	uint8_t id_bit_number;
	uint8_t direction;
	uint8_t last_zero = 0;

	if (searchLastDeviceFlag)
		return false;

	if (!OWReset())
		return false;

	// Not looking like the below wait is needed...
	//waitOnBusy();

	if (searchMode == true) {
		OWWriteByte(OWSearchCmd, 0);
	} else {
		OWWriteByte(OWAlarmSearchCmd, 0);
	}

	for(id_bit_number = 1; id_bit_number < 65; id_bit_number++)
	{
		int searchByte = (id_bit_number - 1)>>3;
		int searchBit = 1<<((id_bit_number-1)&7);

		if (id_bit_number < searchLastDiscrepancy) {
            if ( (ROM_ID[searchByte] & searchBit) > 0)                         
                direction = 1;
            else 
                direction = 0;                                                                                                    
            } else {
                // if equal to last pick 1, if not then pick 0
                if (id_bit_number == searchLastDiscrepancy) 
                    direction = 1;
                else 
                    direction = 0;                    
            }		
		waitOnBusy();  
		begin();
		I2CwriteByte(DS248X_COMMAND_TRIPLET);
		I2CwriteByte(direction ? 0x80 : 0x00);
		end();

		uint8_t status = waitOnBusy();

		uint8_t id = status & DS248X_STATUS_SBR;
		uint8_t comp_id = status & DS248X_STATUS_TSB;
		direction = status & DS248X_STATUS_DIR;

		if (id && comp_id)
		{
			return false;
		}
		else
		{
			if (!id && !comp_id && !direction)
			{
				last_zero = id_bit_number;
			}
		}

		if (direction)
			ROM_ID[searchByte] |= searchBit;
		else
			ROM_ID[searchByte] &= ~searchBit;				
	}
	searchLastDiscrepancy = last_zero;

	if (!last_zero)
		searchLastDeviceFlag = true;
	
	for (uint8_t rom_byte_number = 0; rom_byte_number < 8; rom_byte_number++) {
		dev_addr[rom_byte_number] = ROM_ID[rom_byte_number];
	}	
	return true;
}

/*
 * OWVerify 
 * (Comments from Maxim/Dallas app note 187)
 * The 'VERIFY' operation verifies if a device with a known ROM number is 
 * currently connected to the 1-Wire bus. 
 * 
 * This is accomplished by supplying the ROM number and doing a targeted 
 * search on that number to verify it is present. 
 * 
 * First, set the ROM_NO buffer to the known ROM number. Then set the
 * LastDiscrepancy to 64 (40 hex) and the LastDeviceFlag to 0. Perform the 
 * search operation and then read the ROM_NO result. If the search was 
 * successful and the ROM_NO remains the ROM number that was being searched for, 
 * then the device is currently on the Microlan.
 * @param addr, the 64-bit address being sought
 * @returns Return TRUE : device verified present or FALSE : device not present
 */
boolean DS248X::OWVerify(uint8_t *addr)
{       
	uint8_t found[8];
	uint8_t rom_backup[8];
	boolean result, ldf_backup;
	int8_t ld_backup, lfd_backup;
	uint8_t i;

	// Copy sought device ID to the global ROM_ID array
	for (i = 0; i < 8; i++)
 		ROM_ID[i] = addr[i];

 	// keep a backup copy of the current state
 	for (i = 0; i < 8; i++)
 		rom_backup[i] = ROM_ID[i];

 	ld_backup =  searchLastDiscrepancy;
 	ldf_backup = searchLastDeviceFlag;
 	lfd_backup = searchLastFamilyDiscrepancy;

 	// set search to find the same device
 	searchLastDiscrepancy = 64;
 	searchLastDeviceFlag = false;

 	if (DS248X::OWSearch(found, 1)) 
	{
 		Serial.print("\t\tFound ");
		printArrayInHex(found, 8);
		Serial.println("");
		// check if same device found
 		result = true;
 		for (i = 0; i < 8; i++) 
		{
 			if (rom_backup[i] != found[i]) 
			{
				result = false;
 				break;
 			}
 		}
 	}
 	else {
		result = false;
 	}
	// restore the search state 
 	for (i = 0; i < 8; i++)
 		ROM_ID[i] = rom_backup[i];

 	searchLastDiscrepancy = ld_backup;
 	searchLastDeviceFlag = ldf_backup;
 	searchLastFamilyDiscrepancy = lfd_backup;
 	// return the result of the verify
 	return result;
}


#if DS248X_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t PROGMEM dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Note: this might better be done without the
// table; it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t DS248X::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
	}
	return crc;
}
#else
/****************************************************************** 
 * Dallas 1-wire 16-bit CRC calculation.
 * Compute a Dallas Semiconductor 8 bit CRC directly, slower than lookup
 * table.
 * @param addr array of uint8_t
 * @param len number of members in array for CRC computation
 * @returns Dallas 8-bit CRC
 * 
 *****************************************************************/
uint8_t DS248X::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;
	
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}
#endif


// The OWdataBlock() method still needs work.

/**
*  Sends a block of data and returns the data received in the same array.
*  This method is used when sending a block that contains reads and writes.
*  The 'read' portions of the data block need to be pre-loaded with 0xFF's.
*  It starts sending data from the index at offset 'off' for length 'len'.
*
*  @param  dataBlock  array of data to transfer to and from the 1-Wire Network.
*  @param  off        offset into the array of data to start
*  @param  len        length of data to send / receive starting at 'off'
*
*  For family 10 device first incoming block looks like:
* 
*  read scratchpad cmd
*     /-- 9 bytes of all 1's --\
*  BE FF FF FF FF FF FF FF FF FF
*  BE 1F 00 1C 17 FF FF 06 4D 8D  // Real output
*  returns:
*  BE 01 02 03 04 05 06 07 08 09
*    |------ CRC8 ------------|
*
*  For family 26 device incoming first block looks like:
* 
*  Recall memory cmd
*  /
* B8 01<--- page number
*
* Second incoming block looks like
* read scratchpad cmd
*    /--- 10 bytes of all 1's ---\
* BE FF FF FF FF FF FF FF FF FF FF
* returns:
* BE FF 01 02 03 04 05 06 07 08 09
*       |------ CRC8 ------------|
*/
void DS248X::OWdataBlock(uint8_t dataBlock[], uint8_t off, uint8_t len) {
    int t_off, t_len;
    t_off = off;
    t_len = len;                       
    //Serial.println("\n[I2CBridgeAdapter][dataBlock] dataBlock = ");
    
	begin();
    for (int i = 0; i < t_len; i++) {           
        if (( dataBlock[i] & 0xFF) != 0xFF ) {                 
            t_off++;			
			I2CwriteByte(dataBlock[i]);	
        }
    }
	end();
                       
    //uint8_t recv[] = uint8_t [t_len];    // allocate space for read from device 
    uint8_t recv[len];
	int j = t_off;
	begin();
    for ( int i = 0; j < len; i++, j++ ) {           
        recv[i] = I2CreadByte();           
    }  
	end(); 
	// Need to figure out something other than below... 	     
    //System.arraycopy(recv, 0, dataBlock, t_off, t_len - t_off);                          
}


/* 
  Developed from Maxim Application Note 27.
  The 16-bit routine is new. 
  The 8-bit routine is from http://github.com/paeaetech/paeae/tree/master/Libraries/ds2482/
  
  Copyright (C) 2010 Kairama Inc

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
// 
uint16_t getCRC16( uint8_t *data, uint8_t len)
{
      uint16_t crc=0;
      
      for (uint8_t i=0; i<len;i++) 
      {
            uint8_t inbyte = data[i];
            for (uint8_t j=0;j<8;j++) 
            {
                  uint8_t mix = (byte(crc)^ inbyte) & 0x01;
                  crc = crc >> 1;
                  if (mix) 
                        crc = crc ^ 0xA001;
                  
                  inbyte = inbyte >> 1;
            }
      }
      return crc;
}

