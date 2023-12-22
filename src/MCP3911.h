/*  MCP3911.h
 *  Header file
 *  by Andi Moeller
 *  20231208
 *
 * 
 * 
 * 
 */

//############################################################################
#ifndef MCP3911
#define MCP3911
//############################################################################

#include "ESP32_Controller.h"
#include "FLASH.h"

void IRAM_ATTR MCP3911_DR_isr_handler(void* arg);

void init_SPI2();
void init_MCP3911(MCP3911_calib_data_t*);
void stop_MCP3911();
void init_Interrupt_on_MCP3911_DR();
void stop_Interrupt_on_MCP3911_DR();
void task_MCP3911_DR(void* arg);

//############################################################################

typedef struct { 
  int32_t raw[2];   // raw ADC value, picked from the first buffer location
  int32_t AV[2];    // Average over one buffer
  int32_t F1[2];    // 1st order low pass filtered of raw[]
  int32_t F2[2];    // 1st order low pass filtered of AV[]
} ADC_Result_t;


//############################################################################
#endif  // ifndef MCP3911
//############################################################################
//############################################################################




