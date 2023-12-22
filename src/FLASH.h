/*  FLASH.h
 *  Header file
 *  by Andi Moeller
 *  20231208
 *
 * 
 * 
 * 
 */

//############################################################################
#ifndef FLASH
#define FLASH
//############################################################################


typedef struct {
  int32_t ADC_OCAL0;
  int32_t ADC_OCAL1;
  int32_t ADC_GCAL0;
  int32_t ADC_GCAL1;
} MCP3911_calib_data_t;

//############################################################################


void init_NVS(); // init non volatile storage on flash memory partition
void read_flash_blob(MCP3911_calib_data_t** caliblob);
void write_flash_blob(MCP3911_calib_data_t* caliblob);



//############################################################################
#endif  // ifndef FLASH
//############################################################################
//############################################################################


