#ifndef __DS248X_H__
#define __DS248X_H__

#include <stddef.h>
#include <inttypes.h>

// Chose between a table based CRC (flash expensive, fast)
// or a computed CRC (smaller, slow)
//#define DS248X_CRC8_TABLE 			1

#define DS2482_SEL_CHANNEL         0xC3;   // Selects 1-wire channel in a DS2482-800
#define DS2482_WriteConfigRegCmd   0xD2;
#define DS2482_1WireResetCmd       0xB4;
#define DS2482_1WireSingleBitCmd   0x87;
#define DS2482_1WireWriteByteCmd   0xA5;
#define DS2482_1WireReadByteCmd    0x96;
#define DS2482_1WireTripletCmd     0x78;

#define OWMatchROMCmd           0x55
#define OWSkipRomCmd            0xCC
#define OWSearchCmd             0xF0
#define OWSReadROMCmd           0x33   
#define OWAlarmSearchCmd        0xEC
#define OWConvertTempCmd        0x44
#define OWReadScratchpadCmd     0xBE
#define OWSkipROMCmd            0xCC
#define OWReadScratchPadCmd     0xBE
    
#define DS2482StatusRegister    0xF0   

#define DS248X_COMMAND_RESET	0xF0	// Device reset
#define DS248X_COMMAND_SRP		0xE1 	// Set read pointer

// After a device reset (power-up cycle or initiated by the Device Reset command) 
// the Configuration register reads 00h. When writing to the Configuration register, 
// the new data is accepted only if the upper nibble (bits 7 to 4) is the one's complement 
// of the lower nibble (bits 3 to 0). When read, the upper nibble is always 0h.
//
// bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
//========================================
//  ___  ___  ___  ___
//  1WS  SPU  PPM  APU  1WS  SPU  PPM  APU
    
// All 1-Wire communication commands and the Device Reset command position 
// the read pointer at the Status register for the host processor to read with minimal
// protocol overhead. Status information is updated during the execution of certain 
// commands only. Details are given in the description of the various status bits below.

// bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
//========================================
//  DIR  TSB  SBR  RST   LL   SD  PPD  1WB
//
#define DS248X_POINTER_STATUS		0xF0   
#define DS248X_STATUS_BUSY			0x01	// 1-wire busy
#define DS248X_STATUS_PPD			0x02	// Presence Pulse Detected   
#define DS248X_STATUS_SD			0x04    // Short Detected
#define DS248X_STATUS_LL			0x08   	// Logic Level of the selected 1-wire line
											//	 without initiating any 1-wire communication
#define DS248X_STATUS_RST			0x10   	// If '1', the DS2482 has performed a reset
#define DS248X_STATUS_SBR			0x20    // Single-Bit Result - logic state of 1-wire line sampled at tMSR of a 
                                          	// 	1-wire Single Bit command or the 1st bit of a 1-wire Triplet command   
#define DS248X_STATUS_TSB			0x40    // Triplet Send Bit - reports state of the active 1-wire line sampled 
											// at tMSR of the 2nd bit of a 1-wire Triplet command.  Power-on value = 0
#define DS248X_STATUS_DIR 	 (byte) 0x80 	// Branch Direction taken - search direction chosen by the 3rd bit 
											// of the triplet
#define DS248X_POINTER_DATA			0xE1
#define DS248X_POINTER_CONFIG		0xC3
#define DS248X_CONFIG_APU			0x01
#define DS248X_CONFIG_SPU			0x04
#define DS248X_CONFIG_1WS	 (byte) 0x80
#define POLL_LIMIT					16   // Number of times totest status

// The read-only Status register is the general means for the DS2482 to report 
// bit-type data from the 1-Wire side, 1-Wire busy status and its own reset status 
// to the host processor.
// All 1-Wire communication commands and the Device Reset command position 
// the read pointer at the Status register for the host processor to read with minimal
// protocol overhead. Status information is updated during the execution of certain 
// commands only. Details are given in the description of the various status bits below.

// bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
//========================================
//  DIR  TSB  SBR  RST   LL   SD  PPD  1WB
//
// private #define STATUS_1WB = 0x01;    // 1-wire busy
// private #define STATUS_PPD = 0x02;    // Presence Pulse Detected
// private #define STATUS_SD  = 0x04;    // Short Detected
// private #define STATUS_LL  = 0x08;    // Logic Level of the selected 1-wire line withour initiating any 1-wire communication
// private #define STATUS_RST = 0x10;    // If '1', the DS2482 has performed a reset
// private #define STATUS_SBR = 0x20;    // Single-Bit Result - logic state of 1-wire line sampled at tMSR of a 
//                                                 // 1-wire Single Bit command or the 1st bit of a 1-wire Triplet command   
// private #define STATUS_TSB = 0x40;    // Triplet Send Bit - reports state of the active 1-wire line sampled at tMSR of the 2nd bit
//                                                 // of a 1-wire Triplet command.  Power-on value = 0
// private #define STATUS_DIRECTION_TAKEN  0x80; // Branch Direction taken - search direction chosen by the 3rd bit of the triplet
//   
// DS2482 Commands
//
#define DS248X_COMMAND_WRITECONFIG	0xD2
#define DS248X_COMMAND_RESETWIRE	0xB4
#define DS248X_COMMAND_WRITEBYTE	0xA5
#define DS248X_COMMAND_READBYTE		0x96
#define DS248X_COMMAND_SINGLEBIT	0x87
#define DS248X_COMMAND_TRIPLET		0x78

#define DS248X_ERROR_TIMEOUT		(1<<0)
#define DS248X_ERROR_SHORT			(1<<1)
#define DS248X_ERROR_CONFIG			(1<<2)

class DS248X
{
public:
	DS248X();
	DS248X(uint8_t address);
	//uint8_t getCRC8( uint8_t *addr, uint8_t len);
	//uint16_t getCRC16( uint8_t *data, uint8_t len);
	uint8_t getAddress();
	uint8_t getError();
	uint8_t isConnected();
	void 	setReadPointer(uint8_t readPointer);
	uint8_t readStatus();
	uint8_t readData();
	uint8_t waitOnBusy();
	uint8_t readConfig();
	void	writeConfig(uint8_t config);
	void 	setStrongPullup();
	void 	DS2482Reset();
	uint8_t selectChannel(byte channel);
	void 	clearStrongPullup();
	uint8_t OWReset();
	void 	OWWriteByte(uint8_t data, uint8_t power = 0);
	uint8_t OWReadByte();
	void 	OWWriteBit(uint8_t data, uint8_t power = 0);
	uint8_t OWReadBit();
	void 	OWSkipROM();
	void	OWSelect(uint8_t *address);
	boolean OWSearch(uint8_t *address, bool search_mode);
	void	OWResetSearch();
	void 	OWTargetSearch(uint8_t family_code);
	void 	OWdataBlock(uint8_t dataBlock[], uint8_t off, uint8_t len);
	boolean OWVerify(uint8_t *addr);
	static uint8_t crc8(const uint8_t *addr, uint8_t len);
	void 	write(uint8_t v, uint8_t power = 0);
	uint8_t read(void);
	uint8_t read_bit(void);
	void 	write_bit(uint8_t v);

private:
	void begin();
	uint8_t end();
	void I2CwriteByte(uint8_t);
	uint8_t I2CreadByte();
	uint8_t mAddress;
	uint8_t mError;
	bool doAlarmSearch;
	uint8_t ROM_ID[8];
	int8_t searchLastDiscrepancy;
	int8_t searchLastFamilyDiscrepancy;
	bool searchLastDeviceFlag;
};

#endif