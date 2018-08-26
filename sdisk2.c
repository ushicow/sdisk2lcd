/*------------------------------------------------------------

	DISK II Emulator Farmware (1 of 2) for ATMEGA328P

	version 2.3	2013.10.30 by Koichi Nishida
	require version 2.0 hardware

	Copyright 2013 Koichi NISHIDA
	email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp
	
	Modified by USHIRODA, Atsushi
------------------------------------------------------------*/
/*
hardware information:

use ATMEGA328P AVR.
connect 27MHz (overclock...) crystal to the AVR.
supply 3.3V power.

fuse setting : LOW 11011110

connection:
	<connection to SD card using spi>
	B2: CS (SS)
	B3: DI (MOSI)
	B4: DO (MISO)
	B5: SCLK (SCK)
	D6: LOCK
	
	<connections to APPLE II disk IF>
	B1: WRITE
	C0: PHASE-0
	C1: PHASE-1
	C2: PHASE-2
	C3: PHASE-3
	C4: READ PULSE (through 74HC125 3state)
	D2(INT0): WRITE REQUEST (10K ohm pull up for open collector)
	D3: EJECT
	D4: WRITE PROTECT (through 74HC125 3state)
	D7: DRIVE ENABLE
	
	<others>
	D5: LED
	C5: connect to C6 (RESET) for software reset
	D0: RxD
    D1: TxD
	
	Note that the enable input of the 3state buffer 74HC125,
	should be connected with DRIVE ENABLE.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include "disp.h"

#define nop() __asm__ __volatile__ ("nop")
#define WAIT 1

// write buffer number
#define BUF_NUM 1				// number of buffer
#define BUF_SIZE 350			// size of a buffer
#define OUT_TRACK 0xff			// out of track
#define OUT_SECTOR OUT_TRACK  	// out of sector

// fat buffer size
#define FAT_NIC_SIZE 35

// for selecting SD card
#define SD_CS	2	// PB2 SS	CS

// C prototypes

#define IS_RESET bit_is_set(PIND,3)
#define SOFT_RESET (PORTC &= (~0b00100000))
#define SD_CS_HI (PORTB |= (1<<SD_CS))
#define SD_CS_LO (PORTB &= ~(1<<SD_CS))
#define LED_ON (PORTD |= 0b00100000)
#define LED_OFF (PORTD &= ~(0b00100000))
#define LED_FLASH (PORTD ^= 0b00100000)
#define DISK2_WP_ON (PORTD |= 0b00010000) 
#define DISK2_WP_OFF (PORTD &= ~0b00010000)

// ===== low level SD card functions =====

// cancel read
void cancelRead(void);
// write a byte data to the SD card
void writeByte(unsigned char c);
// read data from the SD card
unsigned char readByte(void);
// wait until finish a command
void waitFinish(void);
// issue SD card command without getting response
void cmd_(unsigned char cmd, unsigned long adr);
// issue SD card command  and wait normal response
void cmd(unsigned char cmd, unsigned long adr);
// get command response from the SD card
unsigned char getResp(void);
// issue command 17 and get ready for reading, cmd17:single block read
void cmd17(unsigned long adr);
// find a file extension

// ===== file manipulate functions =====

void getFileName(unsigned short dir, char *name);
// find a file whose extension is targExt,
// and whose name is targName if withName is true
int findExt(char *targExt, unsigned char *protect,
	unsigned char *targName, unsigned char withName);
// prepare the FAT table on memory
void prepareFat(int i, unsigned short *fat, unsigned short len,
	unsigned char fatNum, unsigned char fatElemNum);
// duplicate FAT for FAT16
void duplicateFat(void);
// write to the SD cart one by one
void writeSD(unsigned long adr, unsigned char *data, unsigned short len);
unsigned short makeFileNameList(unsigned short *list, char *targExt);
// choose a file from file name list
unsigned char chooseFile(void *tempBuff, unsigned char btfExists, unsigned char *filebase);
// initialization called from check_eject

// ===== SDISK II main functions =====

// initialization SD
int SDinit(void);
// write data back to a NIC image
// set write pointer and write back if need
void buffering(void);
void writeBack(void);
void writeBackSub(unsigned char bn, unsigned char sc, unsigned char track);
// buffer clear
void buffClear(void);

// assembler functions
void wait5(unsigned short time);

// ===== data area =====

// SD card information
unsigned long bpbAddr;					// the beginning of BPB Address
unsigned long rootAddr;					// the beginning of Root
unsigned long fatAddr;					// the beginning of FAT
unsigned short bytesPerSector;			// Bytes per Sector
unsigned char sectorsPerCluster;		// sectors per cluster
unsigned char sectorsPerCluster2;		// sectors per cluster by power of 2
unsigned short bytesPerCluster;			// Bytes per Cluster
unsigned short sectorsPerFat;			// sectors per FAT
unsigned long userAddr;					// the beginning of user data
unsigned short fatNic[FAT_NIC_SIZE];
unsigned char prevFatNumNic;
unsigned short wozDir;
unsigned short btfDir;

// DISK II status
volatile unsigned char ph_track;		// 0 - 139
unsigned char sector;					// 0 - 15
unsigned short bitbyte;					// 0 - (8*512-1)
unsigned char prepare;
unsigned char readPulse;
unsigned char magState;
unsigned char protect;
unsigned char formatting;
const unsigned char volume = 0xfe;

// track number is stored on eeprom.
#define EEP_PH_TRACK (uint8_t *)0x0001

// write data buffer
unsigned long trackPos[160];							// data position of each track
unsigned char writeData[BUF_NUM][BUF_SIZE];			// write buffer
unsigned char sectors[BUF_NUM];						// number of sector
unsigned char tracks[BUF_NUM];						// number of track
unsigned long bytePos[26];							// byte position of each sector
unsigned char buffNum;								// current buffer number
unsigned char *writePtr;							// write data pointer
unsigned char doBuffering;							// buffering flag

// a table for head stepper moter movement 
PROGMEM prog_uchar stepper_table[4] = {0x0f,0xed,0x03,0x21};

// a table for translating logical sectors into physical sectors
PROGMEM prog_uchar physicalSector[] = {
	0,13,11,9,7,5,3,1,14,12,10,8,6,4,2,15
};

// LCD messages
PROGMEM const char MSG_INIT[] 	= "  SDISK ][ LCD  ";
PROGMEM const char MSG_VER[]  	= "  FAPPLE2 2018  ";
PROGMEM const char MSG_NODISK[]	= "    NO DISK     ";
PROGMEM const char MSG_SEL[]    = "SELECT WOZ IMAGE";
PROGMEM const char MSG_INSERT[] = " DISK INSERTED  ";
PROGMEM const char MSG_NOFILE[] = " FILE NOT FOUND ";
PROGMEM const char MSG_FAT16[]  = " FAT 16         ";

// buffer clear
void buffClear(void)
{
	int i;
	int j;
	
	for (i = 0; i < BUF_NUM; i++) {
		for (j = 0; j < BUF_SIZE; j++) {
			writeData[i][j] = 0;
		}
	}
	for (i = 0; i < BUF_NUM; i++) {
		sectors[i] = OUT_SECTOR;
		tracks[i] = OUT_TRACK;
	}
}

// cancel read from the SD card
void cancelRead(void)
{
	unsigned short i;
	if (bitbyte<(402*8)) {
		PORTB = 0b00011000;
		for (i=bitbyte; i<(514*8); i++) {
			PORTB = 0b00111000;
			PORTB = 0b00011000;
		}
		bitbyte = 402*8;
	}
}

// write a byte data to the SD card
void writeByte(unsigned char c)
{
	SPDR = c;
	// Wait for transmission complete
	while (!(SPSR & (1<<SPIF)))
		if (IS_RESET) SOFT_RESET;
}

// read data from the SD card
unsigned char readByte(void)
{
	SPDR = 0xFF;
	// Wait for reception complete
	while (!(SPSR & (1<<SPIF)))
		if (IS_RESET) SOFT_RESET;
	// Return Data Register
	return SPDR;
}

// wait until data is written to the SD card
void waitFinish(void)
{
	unsigned char ch;
	do {
		ch = readByte();
	} while (ch != 0xff);
}

// issue a SD card command without getting response
void cmd_(unsigned char cmd, unsigned long adr)
{
	writeByte(0xff);
	writeByte(0x40 + cmd);
	writeByte(adr >> 24);
	writeByte((adr >> 16) & 0xff);
	writeByte((adr >> 8) & 0xff);
	writeByte(adr & 0xff);
	writeByte(0x95);
//	writeByte(0xff);
}

// issue a SD card command and wait normal response
void cmd(unsigned char cmd, unsigned long adr)
{
	unsigned char res;
	do {
		cmd_(cmd, adr);
	} while (((res=getResp())!=0) && (res!=0xff));
}

// get a command response from the SD card
unsigned char getResp(void)
{
	unsigned char ch;
	do {
		ch = readByte();
	} while ((ch&0x80) != 0);
	return ch;
}

// issue command 17 and get ready for reading
void cmd17(unsigned long adr)
{
	unsigned char ch;

	cmd(17, adr);
	do {	
		ch = readByte();
	} while (ch != 0xfe);
}

// get a file name from a directory entry
void getFileName(unsigned short dir, char *name)
{
	unsigned char i;

	cmd(16, 8); // set block length to 8
	cmd17(rootAddr+dir*32);  // read a block
	for (i=0; i!=8; i++) *(name++) = (char)readByte();
	readByte(); readByte(); // discard CRC bytes
}
// find a file whose extension is targExt,
// and whose name is targName if withName is true.
int findExt(char *targExt, unsigned char *protect,
	unsigned char *targName, unsigned char withName)
{
	short i, j;
	unsigned max_file = 512;
	unsigned short max_time = 0, max_date = 0;

	// find extension
	for (i = 0; i != 512; i++) {
		unsigned char name[8];
		unsigned char ext[3];
		unsigned char d;
		unsigned short time;
		unsigned short date;
		
		if (IS_RESET) return 512;
		// check first char
		cmd(16, 1);  // set block length to 1
		cmd17(rootAddr + i * 32);  // read a block
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if ((d == 0x00) || (d == 0x05) || (d == 0x2e) || (d == 0xe5)) continue;
		if (!(((d >= 'A') && (d <= 'Z')) || ((d >= '0') && (d <= '9')))) continue;
		cmd17(rootAddr + i * 32 + 11);  // file attribute
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if (d & 0x1e) continue; // plane file
		if (d == 0xf) continue; // long file name
		// check extension
		cmd(16, 12);
		cmd17(rootAddr + i * 32);
		for (j = 0; j != 8; j++) name[j] = readByte();
		for (j = 0; j != 3; j++) ext[j] = readByte();		
		if (protect) {
			*protect = ((readByte() & 1) << 3);
		} else {
			readByte();
		}
		readByte(); readByte(); // discard CRC bytes
		
		// check time stamp
		cmd(16, 4);
		cmd17(rootAddr + i * 32 + 22);
		time = readByte();
		time += (unsigned short)readByte() * 0x100;
		date = readByte();
		date += (unsigned short)readByte() * 0x100;
		readByte(); readByte(); // discard CRC bytes

		if (memcmp(ext, targExt, 3) == 0) {		
			if ((!withName) || (memcmp(name, targName, 8) == 0)) {
				unsigned short tm = time;
				unsigned short dt = date;

				if ((dt > max_date) || ((dt == max_date) && (tm >= max_time))) {
					max_time = tm;
					max_date = dt;
					max_file = i;
				}
			}
		}
	}

	if ((max_file != 512) && (targName != 0) && (!withName)) {
		unsigned char j;
		cmd(16, 8);
		cmd17(rootAddr + max_file * 32);
		for (j = 0; j < 8; j++) targName[j] = readByte();
		readByte(); readByte();
	}
	return max_file;
	// if 512 then not found...
}

/*
FAT16 structure
0   Byte    name[8];            // file name
8   Byte    extension[3];       // file name extension
11  Byte    attribute;          // file attribute
                                //   bit 4    directory flag
                                //   bit 3    volume flag
                                //   bit 2    hidden flag
                                //   bit 1    system flag
                                //   bit 0    read only flag
12  Byte    reserved;           // use NT or same OS
13  Byte    createTimeMs; 
14  Byte    createTime[2];
16  Byte    createDate[2];
18  Byte    accessDate[2];
20  Byte    clusterHighWord[2];
22  Byte    updateTime[2];
24  Byte    updateDate[2];
26  Byte    cluster[2];         // start cluster number
28  Byte    fileSize[4];        // file size in bytes (directory is always zero)
*/

void testWoz(int dir) {
	unsigned short ft; 
	char message[17] = "                ";
	unsigned long pos = 0;
	unsigned long size = 0;
	unsigned long tmapPos = 0;
	unsigned long trksPos = 0;
	int c;
	
	cmd(16, 2);
	cmd17(rootAddr + dir * 32 + 26);
	ft = readByte();
	ft += (unsigned short)readByte() * 0x100;
	readByte(); readByte(); // discard CRC bytes
	pos = (unsigned long)(ft - 2) * bytesPerCluster;
	pos += 12; // woz header size
	cmd(16, 4);
	do {
		cmd17(userAddr + pos);
		message[0] = readByte();
		message[1] = readByte();
		message[2] = readByte();
		message[3] = readByte();
		readByte(); readByte();
		pos += 4;
		dispStr(LCD_ROW2, message);
_delay_ms(1000);
		cmd17(userAddr + pos);
		size = readByte();
		size += (unsigned long)readByte() * 0x100;
		size += (unsigned long)readByte() * 0x10000;
		size += (unsigned long)readByte() * 0x1000000;
		readByte(); readByte();
		pos += 4;
		if (memcmp(message, "TMAP", 4) == 0) tmapPos = userAddr + pos;
		if (memcmp(message, "TRKS", 4) == 0) trksPos = pos;
		pos += size;
	} while ((tmapPos == 0) || (trksPos == 0));
	for (c = 0; c < 160; ++c) {
		cmd(16, 1);
		cmd17(tmapPos + c);
		trackPos[c] = trksPos + (unsigned long)readByte() * 6656;
		readByte(); readByte();
	}
//	dispStr(LCD_ROW2, "DEBUG1");
}

// prepare a BytePosition table on memory
void prepareBytePos(int dir, unsigned long *pos, unsigned long start, unsigned long len)
{
	unsigned short ft;
	unsigned short fs;
	unsigned long i;
	unsigned short c; // cluster
    unsigned  b;

	cmd(16, 2);		// block size = 2
	cmd17(rootAddr + dir * 32 + 26);    // start cluster number
	ft = readByte();
	ft += (unsigned short)readByte()*0x100;
	readByte(); readByte(); // discard CRC bytes
	fs = ft;
	for (i = 0; i < len; i += 256) {
		c = (start + i) / bytesPerCluster;
		b = (start + i) % bytesPerCluster;
		while (c--) {
			cmd17((unsigned long)fatAddr + (unsigned long)ft * 2);
			ft = readByte();
			ft += (unsigned short)readByte() * 0x100;
			readByte(); readByte(); // discard CRC bytes
			if (ft > 0xfff6) break;
		}
		pos[i >> 7] = (ft - 2) * bytesPerCluster + b;
		ft = fs;
		char msg[17];
		sprintf(msg, "%8ld%8lx", i >> 7, pos[i >> 7]);
		dispStr(LCD_ROW2, msg);
	}
	cmd(16, (unsigned long)256);	
}


void writeSD(unsigned long adr, unsigned char *data, unsigned short len)
{
	unsigned int i;
	unsigned char *buf = &writeData[0][0];

	cmd(16, 512);
	cmd17(adr&0xfffffe00);
	for (i=0; i<512; i++) buf[i] = readByte();
	readByte(); readByte(); // discard CRC bytes
	memcpy(&(buf[adr&0x1ff]), data, len);
	
	SD_CS_HI;
	SD_CS_LO;
				
	cmd(24,adr&0xfffffe00);		// single block write
	writeByte(0xff);
	writeByte(0xfe);
	for (i=0; i<512; i++) writeByte(buf[i]);
	writeByte(0xff);
	writeByte(0xff);
	readByte();
	waitFinish();
	
	SD_CS_HI;
	SD_CS_LO;	
}

/*
void duplicateFat(void)
{
	unsigned short i, j;
	unsigned long adr = fatAddr;
	unsigned char *buf = &writeData[0][0];

	cmd(16, 512);
	for (j=0; j<sectorsPerFat; j++) {
		cmd17(adr);
		for (i=0; i<512; i++) buf[i] = readByte();
		readByte(); readByte(); // discard CRC bytes

		SD_CS_HI;
		SD_CS_LO;	
	
		cmd(24,adr+(unsigned long)sectorsPerFat*512);		
		writeByte(0xff);
		writeByte(0xfe);
		for (i=0; i<512; i++) writeByte(buf[i]);
		writeByte(0xff);
		writeByte(0xff);
		readByte();
		waitFinish();
		adr += 512;

		SD_CS_HI;
		SD_CS_LO;
	}
}

// create a file image
int createFile(unsigned char *name, char *ext, unsigned short sectNum)
{
	unsigned short re, clusterNum;
	unsigned long ft, adr;
	unsigned short d, i;
	unsigned char c, dirEntry[32], at;
	static unsigned char last[2] = {0xff, 0xff};

	if (IS_RESET) return 0;
	
	for (i=0; i<32; i++) dirEntry[i]=0;
	memcpy(dirEntry, (unsigned char *)name, 8);
	memcpy(dirEntry+8, (unsigned char*)ext, 3);
	*(unsigned long *)(dirEntry+28) = (unsigned long)sectNum*512;
	
	// search a root directory entry
	for (re=0; re<512; re++) {
		cmd(16, 1);
		cmd17(rootAddr+re*32+0);
		c = readByte();
		readByte(); readByte(); // discard CRC bytes
		cmd17(rootAddr+re*32+11);
		at = readByte();
		readByte(); readByte(); // discard CRC bytes
		if (((c==0xe5)||(c==0x00))&&(at!=0xf)) break;  // find a RDE!
	}	
	if (re==512) return 0;
	// write a directory entry
	writeSD(rootAddr+re*32, dirEntry, 32);	
	// search the first fat entry
	adr = (rootAddr+re*32+26);
	clusterNum = 0;
	for (ft=2;
		(clusterNum<((sectNum+sectorsPerCluster-1)>>sectorsPerCluster2)); ft++) {
		cmd(16, 2);
		cmd17(fatAddr+ft*2);
		d = readByte();
		d += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes
		if (d==0) {
			clusterNum++;
			writeSD(adr, (unsigned char *)&ft, 2);
			adr = fatAddr+ft*2;
		}
	}
	writeSD(adr, last, 2);
	duplicateFat();
	return 1;
}
*/

// make file name list and sort
unsigned short makeFileNameList(unsigned short *list, char *targExt)
{
	unsigned short i, j, k, entryNum = 0;
	char name1[8], name2[8];

	// find extension
	for (i = 0; i != 512; i++) {
		unsigned char ext[3], d;
		
		if (IS_RESET) return 512;
		// check first char
		cmd(16, 1);
		cmd17(rootAddr + i * 32);
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if ((d == 0x00) || (d == 0x05) || (d == 0x2e) || (d == 0xe5)) continue;
		if (!(((d >= 'A') && (d <= 'Z')) || ((d >= '0') && (d <= '9')))) continue;
		cmd17(rootAddr + i * 32 + 11);
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if (d & 0x1e) continue;
		if (d == 0xf) continue;
		// check extension
		cmd(16, 3);
		cmd17(rootAddr + i * 32 + 8);
		for (j=0; j != 3; j++) ext[j] = readByte();		
		readByte(); readByte(); // discard CRC bytes
		if (memcmp(ext, targExt, 3)==0) {	
			list[entryNum++] = i;
		}
	}
	// sort
	if (entryNum > 1) {
		for (i = 0; i <= (entryNum - 2); i++) {
			for (j=1; j<=(entryNum - i - 1); j++) {
				getFileName(list[j], name1);
				getFileName(list[j - 1], name2);
				if (memcmp(name1, name2, 8) < 0) {
					k = list[j];
					list[j] = list[j - 1];
					list[j - 1]=k;
				}
			}
		}
	}	
	return entryNum;
}

// choose a file from file name list
unsigned char chooseFile(void *tempBuff, unsigned char btfExists, unsigned char *filebase)
{
	unsigned short *list = (unsigned short *)tempBuff;
	unsigned short num = makeFileNameList(list, "WOZ");
	char name[8];
	char filename[] = "filename.WOZ";
	short cur = 0, prevCur = -1;
	unsigned long i;
	char button;

	dispStrP(LCD_ROW1, MSG_SEL);
	// if there is at least one file,
	if (num > 0) {
		// determine first file
		if (btfExists) {
			for (i=0; i < num; i++) {
				getFileName(list[i], name);
				if (memcmp(name, filebase, 8)==0) {
					cur = i;
					break;
				}
			}
		}
		getFileName(list[cur], filename);
		dispStr(LCD_ROW2, filename);
		while (1) {
			dispStr(3, "");
			button = selectButton();
			if (button == 'U') {
				if (cur < (num - 1)) cur++;
			}
			if (button == 'D') {
				if (cur > 0) cur--;
			}
			if (button == 'S') {
				break;
			}
			// display file name
			if (prevCur != cur) {
				prevCur = cur;
			
				getFileName(list[cur], filename);
				dispStr(LCD_ROW2, filename);
			}
		}
		getFileName(list[cur], name);
		memcpy(filebase, name, 8);
		return 1;
	} else {
		dispStrP(LCD_ROW2, MSG_NOFILE);
		return 0;
	}
}

// initialization SD card
int SDinit(void)
{
	unsigned char ch;
	unsigned short i;
	char str[17];
	unsigned char filebase[8];
	unsigned char btfbase[8];
	unsigned char btfExists;
	unsigned char choosen;

	LED_ON;

	//SPI enable, master mode, f/128
	SPCR = ((1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0));

	SD_CS_HI;					// disable CS
	for(i=0; i < 10; i++) writeByte(0xFF);	// Send 10 * 8 = 80 clock
	SD_CS_LO;					// enable CS
	for(i=0; i < 2; i++) writeByte(0xFF);	// Send 2 * 8 = 16 clock

	cmd_(0, 0);	// command 0
 	do {	
		ch = readByte();
	} while (ch != 0x01);
	
	SD_CS_HI;		// disable CS

	dispStrP(LCD_ROW1, MSG_SEL);
	while (1) {
		SD_CS_LO;
		cmd_(55, 0);			// command 55 application command
		ch = getResp();
		if (ch == 0xff) return 0;
		if (ch & 0xfe) continue;
		// if (ch == 0x00) break;
		SD_CS_HI;
		SD_CS_LO;
		cmd_(41, 0);			// command 41 application command
		if (!(ch=getResp())) break;
		if (ch == 0xff) return 0;
		SD_CS_HI;
	}

	//SPI enable, master mode, f/2
	SPCR = ((1<<SPE)|(1<<MSTR));
	SPSR = (1<<SPI2X);

	// BPB address
	cmd(16,5);
	cmd17(54);
	for (i=0; i<5; i++) str[i] = readByte();
	readByte(); readByte();	// discard CRC
	if (memcmp(str, "FAT16", 5) == 0) {
		bpbAddr = 0;
		dispStrP(LCD_ROW1, MSG_FAT16);
		_delay_ms(1000);
	} else {	// MBR
		cmd(16, 4);
		cmd17(0x1c6);
		bpbAddr = readByte();
		bpbAddr += (unsigned long)readByte()*0x100;
		bpbAddr += (unsigned long)readByte()*0x10000;
		bpbAddr += (unsigned long)readByte()*0x1000000;
		bpbAddr *= 512;
		readByte(); readByte(); // discard CRC bytes
		sprintf(str, "   MBR $%08lx", bpbAddr);
		dispStr(LCD_ROW1, str);
		_delay_ms(1000);
	}

	// sectorsPerCluster and reservedSectors
	{
		unsigned short reservedSectors;
		volatile unsigned char k;
		cmd(16, 5);
		cmd17(bpbAddr + 11);
		bytesPerSector = readByte();
		bytesPerSector += (unsigned short)readByte() * 0x100;
		sectorsPerCluster = k = readByte();
		bytesPerCluster = bytesPerSector * sectorsPerCluster;
		sectorsPerCluster2 = 0;
			while (k != 1) {
			sectorsPerCluster2++;
			k >>= 1;
		}
		reservedSectors = readByte();
		reservedSectors += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes	
		// sectorsPerCluster = 0x40 at 2GB, 0x10 at 512MB
		// reservedSectors = 2 at 2GB
		fatAddr = bpbAddr + bytesPerSector * reservedSectors;
		sprintf(str, "   FAT $%08lx", fatAddr);
		dispStr(LCD_ROW1, str);	
		_delay_ms(1000);	
	}

	// sectorsPerFat and rootAddr
	{
		unsigned short rootEntryCount;
		cmd(16, 2);
		cmd17(bpbAddr + 17);
		rootEntryCount = readByte();
		rootEntryCount += (unsigned short)readByte() * 0x100;
		readByte(); readByte(); // discard CRC bytes				
		cmd17(bpbAddr + 22);
		sectorsPerFat = readByte();
		sectorsPerFat += (unsigned short)readByte() * 0x100;
		readByte(); readByte(); // discard CRC bytes		
		rootAddr = fatAddr + (unsigned long)sectorsPerFat * 2 * bytesPerSector;
		userAddr = rootAddr + (unsigned long)rootEntryCount * 32;
	}
	
	// find "BTF" boot file
	btfDir = findExt("BTF", (unsigned char *)0, btfbase, 0);
	btfExists = (btfDir != 512);

    // choose a file from file list
	choosen = chooseFile(&writeData[0][0], btfExists, btfbase);

	if (btfExists || choosen) memcpy(filebase, btfbase, 8);

	// find "WOZ" extension
	wozDir = findExt("WOZ", &protect, filebase, btfExists || choosen);
	if (wozDir == 512) return 0;

	testWoz(wozDir);
	
	prevFatNumNic = 0xff;
	cmd(16, (unsigned long)256);
	SPCR = 0;					// disable spi
	LED_OFF;
	
	return 1;
}

// move head
ISR(PCINT1_vect)
{
	if (bit_is_set(PIND, 7)) return;
	
	unsigned char stp;
	static unsigned char prevStp = 0;

	stp = (PINC & 0b00001111);
	if (stp != prevStp) {
		prevStp = stp;
		unsigned char ofs =
			((stp==0b00001000)?2:
			((stp==0b00000100)?4:
			((stp==0b00000010)?6:
			((stp==0b00000001)?0:0xff))));
		if (ofs != 0xff) {
			ofs = ((ofs+ph_track)&7);
			unsigned char bt = pgm_read_byte_near(stepper_table + (ofs>>1));
			prevStp = stp;
			if (ofs&1) bt &= 0x0f; else bt >>= 4;
			ph_track += ((bt & 0x08) ? (0xf8 | bt) : bt);
			if (ph_track > 196) ph_track = 0;	
			if (ph_track > 139) ph_track = 139;
		}
	}
}

int main(void)
{
	char tracknum[] = " TRACK #00.00   ";
	char subtrack[4][3] = {"00", "25", "50", "75"};
	char pretrk = 0xff;

	DDRB =  0b00101100;	
	DDRC =  0b00110000;
	DDRD =  0b00110000;

	PORTB = 0b00010001;
	PORTC = 0b00100000;
	PORTD = 0b01001001;

	sei();

	eeprom_busy_wait();
	ph_track = eeprom_read_byte(EEP_PH_TRACK);
	if (ph_track > 196) ph_track = 0;	
	if (ph_track > 139) ph_track = 139;

	// pc interrupt for head move
	PCMSK1 = 0b00001111;
	PCICR = (1<<PCIE1);

	unsigned long i;

	dispInit();
	_delay_ms(100);
	dispStrP(LCD_ROW1, MSG_INIT);
	dispStrP(LCD_ROW2, MSG_VER);
	_delay_ms(1000);
	dispStr(LCD_CLEAR, "");

	dispStrP(LCD_ROW1, MSG_NODISK);
	// wait long low of eject
	for (i=0; i!=0x80000;i++) {	
		if (IS_RESET) {
			i=0; continue;
		}
	}
	// now SD card has been inserted completely

	// timer0 (4u sec)
	OCR0A = 108;
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS00);
	
	MCUCR = 0b00000010;
	
	// int0 falling edge interrupt for write request
	// int1 rising edge interrupt for eject
	EICRA = 0b00001110;

    dispStrP(LCD_ROW1, MSG_SEL);

	if (SDinit()) {	
		// initialize variables
		readPulse = 0;
		magState = 0;
		prepare = 1;
		bitbyte = 0;
		sector = 0;
		buffNum = 0;
		formatting = 0;
		writePtr = &(writeData[buffNum][0]);
		buffClear();
		TIMSK0 |= (1<<OCIE0A);	// on timer
		EIMSK |= (1<<INT0);	// on int0
		EIMSK |= (1<<INT1);	// on int1
		SPCR = 0; 				// off spi
	} else while (1) ;
	
	dispStrP(LCD_ROW1, MSG_INSERT);
	while (1) {
		if (bit_is_set(PIND, 7)) { 				// disable drive
			LED_OFF;
		} else { 									// enable drive
			LED_ON;
			if (bit_is_set(PIND,6)||protect||(PINC&2))
				DISK2_WP_ON;
			else
				DISK2_WP_OFF;
			if (prepare) {
				TIMSK0 &= ~(1<<OCIE0A);			// disable timer0
				sector = ((sector >= 25) ? 0 : sector + 1);
dispStr(LCD_ROW2, "DEBUG2");
//				unsigned char trk = (ph_track>>2);
//				if (!(((sectors[0]^sector)|(tracks[0]^trk)) &
//					((sectors[1]^sector)|(tracks[1]^trk)) &
//					((sectors[2]^sector)|(tracks[2]^trk)) &
//					((sectors[3]^sector)|(tracks[3]^trk)) &
//					((sectors[4]^sector)|(tracks[4]^trk))))
//					writeBack();
				SPCR = ((1<<SPE)|(1<<MSTR));		// enable spi
				if (ph_track != pretrk) {
					sprintf(tracknum, " TRACK #%02d.%s   ", ph_track >> 2, subtrack[ph_track % 3]);
					dispStr(LCD_ROW1, tracknum);
					prepareBytePos(wozDir, bytePos, trackPos[ph_track] + sector * 256, 6656);
					pretrk = ph_track;
				}
				sprintf(tracknum, "%4d    %8lx", sector, bytePos[sector]);
				dispStr(LCD_ROW2, tracknum);
				cmd17(bytePos[sector]);
				bitbyte = 0;
				prepare = 0;
				SPCR = 0;							// disable SPI
				TIMSK0 |= (1<<OCIE0A);				// enable timer0
			}
			if (doBuffering) {
				doBuffering = 0;
				buffering();
			}
		}
	}
}

void writeBackSub(unsigned char bn, unsigned char sc, unsigned char track)
{
	unsigned char c;
	unsigned short i;
	unsigned short long_sector = (unsigned short)track*16+sc;
	unsigned short long_cluster = long_sector>>sectorsPerCluster2;
	unsigned char fatNum = long_cluster/FAT_NIC_SIZE;
	unsigned short ft;

	TIMSK0 &= ~(1<<OCIE0A);		// disable timer0
	SPCR = ((1<<SPE)|(1<<MSTR));	// enable spi

	// BPB address
	if (fatNum != prevFatNumNic) {
		prevFatNumNic = fatNum;
//		prepareFat(wozDir, fatNic,
//			(560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_SIZE);
	}
	ft = fatNic[long_cluster%FAT_NIC_SIZE];
	
	SD_CS_HI;
	SD_CS_LO;

	cmd(24, (unsigned long)userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
		+ (unsigned long)(long_sector&(sectorsPerCluster-1)))*512);

	writeByte(0xff);
	writeByte(0xfe);
	
	// 22 ffs
	for (i = 0; i < 22; i++) {
		writeByte(0xff);
	}

	// sync header
	writeByte(0x03);
	writeByte(0xfc);
	writeByte(0xff);
	writeByte(0x3f);
	writeByte(0xcf);
	writeByte(0xf3);
	writeByte(0xfc);
	writeByte(0xff);
	writeByte(0x3f);
	writeByte(0xcf);
	writeByte(0xf3);
	writeByte(0xfc);

	// address header
	writeByte(0xd5);
	writeByte(0xAA);
	writeByte(0x96);
	writeByte((volume>>1)|0xaa);
	writeByte(volume|0xaa);
	writeByte((track>>1)|0xaa);
	writeByte(track|0xaa);
	writeByte((sc>>1)|0xaa);
	writeByte(sc|0xaa);
	c = (volume^track^sc);
	writeByte((c>>1)|0xaa);
	writeByte(c|0xaa);
	writeByte(0xde);
	writeByte(0xAA);
	writeByte(0xeb);

	// sync header
	writeByte(0xff);	
	writeByte(0xff);
	writeByte(0xff);
	writeByte(0xff);
	writeByte(0xff);

	// data
	for (i = 0; i < 349; i++) {
		c = writeData[bn][i];
		writeByte(c);
	}
	for (i = 0; i < 14; i++) {
		writeByte(0xff);
	}
	for (i = 0; i < 96; i++) {
		writeByte(0);
	}
	writeByte(0xff);
	writeByte(0xff);
	readByte();
	waitFinish();

	SD_CS_HI;
	SD_CS_LO;	

	SPCR = 0;							// disable spi
	TIMSK0 |= (1<<OCIE0A);				// enable timer0
}

// write back into the SD card
void writeBack(void)
{
	unsigned char i, j;

	for (j=0; j<BUF_NUM; j++) {
		if (sectors[j]!=0xff) {
			for (i=0; i<BUF_NUM; i++) {
				if (sectors[i] != 0xff)
					writeBackSub(i, sectors[i], tracks[i]);
				sectors[i] = 0xff;
				tracks[i] = 0xff;
				writeData[i][2]=0;
			}
			buffNum = 0;
			writePtr = &(writeData[buffNum][0]);
			break;
		}
	}
}

// set write pointer and write back if need
void buffering(void)
{
	static unsigned char sec;

	if (writeData[buffNum][2]==0xAD) {
		if (!formatting) {
			sectors[buffNum]=sector;
			tracks[buffNum]=(ph_track>>2);
			sector=((((sector==0xf)||(sector==0xd))?(sector+2):(sector+1))&0xf);
			if (buffNum == (BUF_NUM-1)) {
				// cancel reading
				cancelRead();
				writeBack();
				prepare = 1;
			} else {
				buffNum++;
				writePtr = &(writeData[buffNum][0]);
			}
		} else {
			sector = sec;
			formatting = 0;
			if (sec == 0xf) {
				// cancel reading
				cancelRead();
				prepare = 1;
			}
		}
	} if (writeData[buffNum][2]==0x96) {
		sec = (((writeData[buffNum][7]&0x55)<<1) | (writeData[buffNum][8]&0x55));
		formatting = 1;
	}
}

// reset when eject
ISR(INT1_vect)
{
	unsigned long i;

	// wait long high of eject
	for (i=0; i!=0x80000;i++)
		if (bit_is_clear(PIND,3)) return;
	// now SD card has been removed completely

	// record track number on eeprom
	eeprom_busy_wait();
	eeprom_write_byte (EEP_PH_TRACK, ph_track); 

	// and reset!
	SOFT_RESET;
}
	
	