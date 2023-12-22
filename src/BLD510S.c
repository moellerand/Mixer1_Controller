/*  BLD510S.c
 *  A simple application for controlling a BLDC motor'
 *  by Andi Moeller
 *  20231122
 *  20231210
 * 
 * 
 * 
 */


//############################################################################
//############################################################################
// custom header files
#include "ESP32_Controller.h"
#include "BLD510S.h"
#include "UART.h"


//############################################################################
// prototypes
static void write_BLD510S(uint16_t reg_addr, uint16_t reg_bits);
static uint16_t MODBUS_CRC16( const unsigned char *, unsigned int len); 


//############################################################################
typedef struct { 
    int8_t      dev_address; 
    int8_t      function_code;
    int16_t     regi;
    int16_t     param;
    int16_t     CRC16;
} MODBUS_command_t;

typedef struct {
	uint16_t address; 
	uint16_t setting;
} BLD510S_reg;

BLD510S_reg status    = { 0x8000, 0x020B }; // Bits: 0->EN, 1->Forward/Rev, 2->Brake, 3->NW, 4->MDX, 5->X12, 6->KH,..., 8:11->NoPolePairs, 12:15->HallAngle
// NW = 1: digital control, MDX and X12 have no function
BLD510S_reg max_speed = { 0x8001, 0x0000 }; // maximum speed for analoge speed regulation
BLD510S_reg start_tork_speed = { 0x8002, 0x0440 }; // Byte0: start tork, Byte1: sensorless start speed (no effect)
BLD510S_reg acceleration = { 0x8003, 0xF4F4 }; // Byte0: acceleration time, Byte1: deceleration time [0.1s] (no effect)
BLD510S_reg max_current = { 0x8004, 0x0000 }; //	Byte0: sensorless=0x10 - sensored= 0x0F, Byte1 MaxCurrent Fuse  (default = 0x90 = 13A)
BLD510S_reg speed = { 0x8005, 0x0000 }; // maximum speed for digital control
BLD510S_reg braking_force = { 0x8006, 0x03FF }; //	0-1023
BLD510S_reg read_speed = { 0x8018, 0x0}; //	
BLD510S_reg read_error = { 0x801B, 0x0 }; //	

//############################################################################
//############################################################################
//############################################################################
//# see BLD-510S documentation (by www.OMC-Stepperonline.com)
void BLD510S_init() {
  write_BLD510S(0x05, 0);       // set speed to zero
  write_BLD510S(0x00, 0x020B);  // number of pole pairs | EN = 1, Forward/Rev. = 2, Break = 4, NW = 8
  //write_BLD510S(0x02, 0x0440);  // Sensorless_Start_Speed | StartTorque (no effect)
  //write_BLD510S(0x03, 0xF4F4);  // acceleration / deceleration (no effect) [0.1 s]
  //write_BLD510S(0x04, 0x0F90);  // Byte0: sensorless=0x10 - sensored= 0x0F, Byte1 MaxCurrent Fuse  (default = 0x90 = 13A)
  write_BLD510S(0x06, 1023);    // braking force
}

//############################################################################
//# see BLD-510S documentation (by www.OMC-Stepperonline.com)
void BLD510S_set_speed(int32_t rpm) { 
	static int32_t rpm_last = 0; 
	int32_t rpm_accl;
	const int32_t accl_step = 20;
	if(1200 < rpm) rpm = 1200;    // limit speed
	if(   0 > rpm) rpm =    0;    // limit speed 
  //if(-500 > rpm) rpm = -500;    // limit speed

	if(0 == rpm) { 
		write_BLD510S(0x00, 0x020F); 	// break
		rpm_last = 0;
		return;												// and leave
	}
	if(0 == rpm_last) {							// starting from stop
		write_BLD510S(0x00, 0x020B);  // release break
	} 
	if(rpm < rpm_last) { 						// deceleration needed
		write_BLD510S(0x05, rpm);   	// write new speed
		rpm_last = rpm;
		return;												// and leave
	}
	if(rpm > rpm_last) {						// acceleration needed
		if(rpm_last < 250) { 
			rpm_accl = 250;
			if(rpm_accl > rpm) rpm_accl = rpm; 
		} else {
			rpm_accl = rpm_last;
		}
		while(rpm_accl < rpm) {
			rpm_accl += accl_step;				
			if(rpm_accl > rpm) rpm_accl = rpm; 
			write_BLD510S(0x05, rpm_accl);  	// write new speed
		}
	}
	rpm_last = rpm;
}

//############################################################################
//# see BLD-510S documentation (by www.OMC-Stepperonline.com)
void write_BLD510S(uint16_t reg_addr, uint16_t reg_bits) {
  uint8_t b[8];
  uint16_t crc;
  b[0] = 0x01;
  b[1] = 0x06;
  b[2] = 0x80;
  b[3] = reg_addr & 0xFF;
  b[4] = reg_bits & 0xFF;
  b[5] = (reg_bits >> 8) & 0xFF;
	crc = MODBUS_CRC16(b, 6);
  b[6] =  crc & 0xFF;
  b[7] = (crc >> 8) & 0xFF;
  //for(int16_t i = 0; i < 8; i++) send_UART0(char s[8], sprintf(s, "%02X", b[i]));
  send_UART1((char*)b, 8);
	vTaskDelay(130 / portTICK_PERIOD_MS);
}

//############################################################################
//# see BLD-510S documentation (by www.OMC-Stepperonline.com)
void BLD510S_read_speed() { // a read sequence of 0x01 0x03 0x80 0x18 0x00 0x00 CRC16 sends unrecognizable answer
  const char b[8] = {0x1, 0x3, 0x80, 0x18, 0x00, 0x1, 0x2D, 0xCD };
  /*
	char b[8];
  uint16_t crc;
  b[0] = 0x01;
  b[1] = 0x03;
  b[2] = 0x80;
  b[3] = 0x1B;
  b[4] = 0x00;
  b[5] = 0x01; // number of 16bit values to query
	crc = MODBUS_CRC16(b, 6);
  b[6] =  crc & 0xFF;
  b[7] = (crc >> 8) & 0xFF;
	*/
	
	// No documented in the manual:
	// Response format in hex: 0x01-0x03-NO-SL-SH-CRCL-CRCH
	// NO: number of bytes (payload)  sent in response
	// SL-SH: low-byte, high-byte of speed in 12*turns/second
	// CRCL-CRCH: low-byte, high-byte of MODBUS CRC16 
  //char s[32]; for(int16_t i = 0; i < 8; i++) send_UART0(s, sprintf(s, "%02X.", b[i]));
  send_UART1(b, 8);
	vTaskDelay(130 / portTICK_PERIOD_MS);
}

//############################################################################
//# see BLD-510S documentation (by www.OMC-Stepperonline.com)
void BLD510S_read_status() { // a read sequence of 0x01 0x03 0x80 0x18 0x00 0x00 CRC16 sends unrecognizable answer
  const uint8_t b[8] = {0x1, 0x3, 0x80, 0x1B, 0x00, 0x1, 0xDD, 0xCD };
  /*
	uint16_t crc;
  b[0] = 0x01;
  b[1] = 0x03;
  b[2] = 0x80;
  b[3] = 0x18;
  b[4] = 0x00;
  b[5] = 0x01; // number of 16bit values to query
	crc = MODBUS_CRC16(b, 6);
  b[6] =  crc & 0xFF;
  b[7] = (crc >> 8) & 0xFF;
	// No documented in the manual:
	// Response format in hex: 0x01-0x03-NO-SL-SH-CRCL-CRCH
	// NO: number of bytes (payload)  sent in response
	// SL-SH: low-byte, high-byte of speed in 12*turns/second
	// CRCL-CRCH: low-byte, high-byte of MODBUS CRC16 
	*/
  //char s[32]; for(int16_t i = 0; i < 8; i++) send_UART0(s, sprintf(s, "%02X.", b[i]));
  send_UART1((char*)b, 8);
	vTaskDelay(130 / portTICK_PERIOD_MS);
}

//############################################################################
//#
uint16_t MODBUS_CRC16( const unsigned char *buf, unsigned int len ) {
  static const uint16_t table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };
	uint8_t xor = 0;
	uint16_t crc = 0xFFFF;

	while( len-- ) {
		xor = (*buf++) ^ crc;
		crc >>= 8;
		crc ^= table[xor];
	}
	return crc;
}


//############################################################################
//############################################################################
//############################################################################
//############################################################################
