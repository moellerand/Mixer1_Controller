/*  MCP3911.c
 *  by Andi Moeller
 *  20231122
 *
 * 
 * 
 * 
 */

//############################################################################

#include "ESP32_Controller.h"
#include "MCP3911.h"
#include "FLASH.h"

#include <hal/spi_types.h>
#include <driver/spi_master.h>
#include <soc/spi_periph.h>

//more information for reverse-engineering the SPI API may be found here:
//#include <spi_ll.h>
//#include <spi_hal.h>
//#include <soc/spi_struct.h>
//#include <soc/spi_reg.h>
//#include "soc/soc.h"
//#include "soc/periph_defs.h"
//#include "soc/spi_caps.h"

//############################################################################
typedef union { 
  int64_t i64;
  int32_t  i32[2];
  uint8_t  byte[8];
} rxbuffer_t;

typedef union { 
  int32_t  i32;
  uint8_t  byte[4];
} rxvalue_t;

void send_MCP3911(uint8_t address, uint32_t spidat);
int16_t read_MCP3911(uint8_t address);

//############################################################################

spi_device_handle_t MCP3911_dev;
spi_transaction_t MCP3911_transmit = {0 }; 
DRAM_ATTR spi_dev_t *host = (spi_dev_t *)REG_SPI_BASE(2); // = SPI2->host;

QueueHandle_t GPIO_evt_queue = NULL;
ADC_Result_t ADC_Result;  //
MCP3911_calib_data_t * ADC_CalibData; // 

#define MCP3911_buf_length 122 // see init_MCP3911 for understanding the sample rate of ~488 Hz and GPIO event rate of ~4 Hz


//############################################################################
//#
void init_Interrupt_on_MCP3911_DR() {
  gpio_set_intr_type(PIN_ADC_DR, GPIO_INTR_DISABLE);  // interrupt disable during installation
  gpio_set_direction(PIN_ADC_DR, GPIO_MODE_INPUT);    // set GPIO for input mode
  gpio_set_pull_mode(PIN_ADC_DR, GPIO_PULLUP_ONLY);   // enable pull-up mode
  gpio_set_intr_type(PIN_ADC_DR, GPIO_INTR_NEGEDGE);  // io_conf.intr_type = GPIO_INTR_NEGEDGE;

  GPIO_evt_queue = xQueueCreate(1, sizeof(rxbuffer_t *));   // 1:= task must terminate before new data is available (ping-pong)
  xTaskCreate(task_MCP3911_DR, "task_MCP3911_DR", 2048, NULL, 10, NULL);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);  // priority level 3 is the highest level working  
  gpio_isr_handler_add(PIN_ADC_DR, MCP3911_DR_isr_handler, (void*) PIN_ADC_DR);
  
}

//############################################################################
//#
void stop_Interrupt_on_MCP3911_DR() {
  gpio_uninstall_isr_service();
}

//############################################################################
//#
void task_MCP3911_DR(void* arg) { // runs @ ~4 Hz
  static uint32_t bufidx;
  static volatile rxbuffer_t * SPIbuf;
  static volatile int32_t MCP3911_CHAN_buffer[2][MCP3911_buf_length]; // ADC results
  static rxvalue_t r0, r1, s0, s1;    // raw data
  static int32_t avrge0, avrge1; // averaged data
  static int32_t d0, d1;         // high-pass filtered data (just for local calculations)
  static int32_t f0 = 0, f1 = 0; // averager for 1st order filtered low pass
  static int32_t g0 = 0, g1 = 0; // averager for very low pass filtered data
  
  for(;;) {
    if(xQueueReceive(GPIO_evt_queue, &SPIbuf, portMAX_DELAY)) {  
      for(bufidx = 0; bufidx < MCP3911_buf_length; bufidx++) {
        // byte access on IRAM_ATTR data stalls the CPU !
        s0.i32 = SPIbuf[bufidx].i32[0];
        s1.i32 = SPIbuf[bufidx].i32[1];
        r0.byte[3] = s0.byte[0];        // pick and place bytes from SPI stream
        r0.byte[2] = s0.byte[1];        // 
        r0.byte[1] = s0.byte[2];        // 
        r1.byte[3] = s0.byte[3];        // 
        r1.byte[2] = s1.byte[0];        // 
        r1.byte[1] = s1.byte[1];        // 
        MCP3911_CHAN_buffer[0][bufidx] = r0.i32 >>= 12; // shift and sign extend (20bit data is remaining)
        MCP3911_CHAN_buffer[1][bufidx] = r1.i32 >>= 12; // store data in buffer
      }
      avrge0 = 0; avrge1 = 0;
      for(bufidx = 0; bufidx < MCP3911_buf_length; bufidx++) {
        avrge0 += MCP3911_CHAN_buffer[0][bufidx]; // calculate the average of one buffer 
        avrge1 += MCP3911_CHAN_buffer[1][bufidx]; // ""
        d0 = MCP3911_CHAN_buffer[0][bufidx] - (f0>>7); // substract filtered signal
        d1 = MCP3911_CHAN_buffer[1][bufidx] - (f1>>7); // dn = deviation to filtered signal 
        f0 += d0;   //  1st order low pass filtered data         
        f1 += d1;   //
      }
      // debug data
      ADC_Result.raw[0] = r0.i32;  // ??????
      ADC_Result.raw[1] = r1.i32;  // ??????
      ADC_Result.AV[0] = avrge0 / MCP3911_buf_length;  // ??????
      ADC_Result.AV[1] = avrge1 / MCP3911_buf_length;  // ??????
      ADC_Result.F1[0] = f0>>7;  // ??????
      ADC_Result.F1[1] = f1>>7;  // ??????
      d0 = ADC_Result.AV[0] - (g0>>4); g0 += d0;
      d1 = ADC_Result.AV[1] - (g1>>4); g1 += d1;
      ADC_Result.F2[0] = g0>>4;  // ??????
      ADC_Result.F2[1] = g1>>4;  // ??????
      
    }
  }
}

//############################################################################
//#
void init_SPI2() {  // connected to MCP3911 ADC
  char s[20];
  esp_err_t ret;
  spi_bus_config_t buscfg = { };
  buscfg.miso_io_num = SPI2_IOMUX_PIN_NUM_MISO;
  buscfg.mosi_io_num = SPI2_IOMUX_PIN_NUM_MOSI;
  buscfg.sclk_io_num = SPI2_IOMUX_PIN_NUM_CLK;
  buscfg.flags = SPICOMMON_BUSFLAG_NATIVE_PINS;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4; //?
  //buscfg.isr_cpu_id = 
  buscfg.intr_flags = ESP_INTR_FLAG_IRAM;

  //Initialize the SPI bus
  ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH1);
  //if(ESP_OK != ret) { send_UART0(s, sprintf(s,"ERR buscfg %X\n\r", ret));}

  spi_device_interface_config_t devcfg = { };
  devcfg.address_bits = 8;
  devcfg.clock_source = SPI_CLK_SRC_DEFAULT;    // Select SPI clock source
  devcfg.clock_speed_hz = SPI_MASTER_FREQ_26M;  // SPI Clock: 8M,10M,13M,16M,20M,26M->OK with MCP3911, 40M->NOT OK without adjusted delay
                                                // note: MCP3911 SPI clock is rated 20MHz max
  devcfg.input_delay_ns = 18;                   // adjust delayed input reading for higher speed:  12..23ns to allow for 40MHz operation of MCP3911
  devcfg.mode = 3;                              // SPI mode 3
  devcfg.spics_io_num = SPI2_IOMUX_PIN_NUM_CS;  // CS pin
  devcfg.queue_size = 7;                        // We want to be able to queue 7 transactions at a time (makes no sense here)
  devcfg.pre_cb = NULL; //Specify pre-transfer callback function
  devcfg.post_cb = NULL;//Specify post-transfer callback function

  //Attach the A2D converter MCP3911 to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &MCP3911_dev);
  //if(ESP_OK != ret) { send_UART0(s, sprintf(s, "ERR devcfg %X\n\r", ret));}
}

//############################################################################
//#
void init_MCP3911(MCP3911_calib_data_t* caliblob) {  // ADC is used in 24 bit mode
  /********************************************************
  ESP32 and MCP3911 have different endianness. 
  ESP32 stores integer numbers with low byte at the lower 
  memory address. (= Little-endian). 
  MCP3911 in contrast uses Big-endian.
  The functions send_MCP3911() and read_MCP3911...() swap 
  bytes for aligning the transfered bytes in the right way.
  **********************************************************/
  #define MCP3911_CHAN0_ADDR  (0x00<<1) // read-only 
  #define MCP3911_CHAN1_ADDR  (0x03<<1)
  #define MCP3911_MOD_ADDR    (0x06<<1) // Writting to this 8-bit register affects the next register. 
  #define MCP3911_PHASE_ADDR  (0x07<<1)
  #define MCP3911_GAIN_ADDR   (0x09<<1) // Writting to this 8-bit register affects the next register. 
  #define MCP3911_STATUS_ADDR (0x0A<<1)
  #define MCP3911_CONFIG_ADDR (0x0C<<1)
  #define MCP3911_OFFCAL0_ADDR   (0x0E<<1)
  #define MCP3911_GAINCAL0_ADDR  (0x11<<1)
  #define MCP3911_OFFCAL1_ADDR   (0x14<<1)
  #define MCP3911_GAINCAL1_ADDR  (0x17<<1)
  static uint32_t spidat;   // High byte of spidat goes to high byte in MCP3911 register.
  

  //memset(&MCP3911_transmit, 0, sizeof(MCP3911_transmit));
  MCP3911_transmit.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  
  // Initialize MCP3911 ADC after a warm start. 
  // Single byte operations are necessairy because READ and WRITE bits may be corrupted.
  MCP3911_transmit.length   = 8;
  MCP3911_transmit.rxlength = 8;
  MCP3911_transmit.addr = MCP3911_CONFIG_ADDR+2;
  MCP3911_transmit.tx_data[0] = 0xC0; // set RESET and CLKEXT bits
  spi_device_transmit(MCP3911_dev, &MCP3911_transmit);
  MCP3911_transmit.addr = MCP3911_STATUS_ADDR+2;
  MCP3911_transmit.tx_data[0] = 0xE0; // set READ and WRITE bits
  spi_device_transmit(MCP3911_dev, &MCP3911_transmit);

  // Initialize MCP3911 ADC after a cold boot. 
  MCP3911_transmit.length   = 16;
  MCP3911_transmit.rxlength = 16;
  // High byte of spidat goes to high byte in MCP3911 register. 
  // send_MCP3911() is dealing with byte order
  spidat  =  2<<14;  // PRE: set AMCLK = MCLK/X  0-> X=1 (default), 1-> X=2, 2-> X=4, 3-> X=8
  spidat |=  6<<11;  // OSR: 7-> 4096, 6->2048, 5->1024, 4->512, 3->256, 2->128, 1->64, 0->32 (OverSampling Rate)
  spidat |=  0<<9;   // DITHER: 00 = Dithering turned off
  spidat |=  0<<8;   // AZ_FREQ: 0 = Auto-zeroing algorithm running at lower speed (default)
  spidat |=  3<<6;   // RESET: 11 = Both CH0 and CH1 ADC are in Reset mode
  spidat |=  0<<4;   // SHUTDOWN: 00 = Neither channel is in shutdown (default)
  spidat |=  0<<2;   // VREFEXT: 0 = Internal voltage reference enabled (default)
  spidat |=  0<<1;   // CLKEXT: 0 = crystal resonator connected 1 = External clock drive by MCU on OSC1 pin
  send_MCP3911(MCP3911_CONFIG_ADDR, spidat);
  
  // PHASE
  spidat = 0;       // MCP3911_PHASE
  send_MCP3911(MCP3911_PHASE_ADDR, spidat);

  // GAIN  8-bit register
  spidat  =  1<<6;  // Boost: ADC power setting 0: x0.5, 1: x0.66, 2: x1.00, 3: x2.00
  spidat |=  1<<3;  // PGA_CH1:
  spidat |=  1<<0;  // PGA_CH0: Differential input voltage range setting 0->+-600mV, 1->+-300mV, 2->+-150mV, 3->+-75mV
  send_MCP3911(MCP3911_GAIN_ADDR, spidat);

  // STATUS
  spidat  =  0<<14;  // MODOUT: Modulator outputs are not used.
  spidat |=  0<<13;  // not used
  spidat |=  1<<12;  // DR_HIZ: Push-pull output on DR pin
  spidat |=  0<<10;  // DRMODE: The lagging ADC controlls the DR signal
  spidat |=  3<<8;   // DRSTATUS: default
  spidat |=  2<<6;   // READ: 2 = Address counter loops on register types (default)
  spidat |=  1<<5;   // WRITE: 1 = Address counter loops on entire register map (default)
  spidat |=  3<<3;   // WIDTH: 0-> Both channels are in 16-bit mode, 3-> Both channels are in 24-bit mode
  spidat |=  1<<2;   // OFFCAL: 1-> calibration enabled  0-> no calibration
  spidat |=  1<<1;   // GAINCAL: 1-> calibration enabled  0-> no calibration
  send_MCP3911(MCP3911_STATUS_ADDR, spidat);  
  
  // CONFIG set sample rate to 488Hz
  spidat  =  2<<14;  // PRE: set AMCLK = MCLK/X  0-> X=1 (default), 1-> X=2, 2-> X=4, 3-> X=8
  spidat |=  6<<11;  // OSR: 7-> 4096, 6->2048, 5->1024, 4->512, 3->256, 2->128, 1->64, 0->32 (OverSampling Rate)
  spidat |=  2<<9;   // DITHERing: 0-> off, 1->minimum, 2->medium, 3->maximum
  spidat |=  0<<8;   // AZ_FREQ: 0 = Auto-zeroing algorithm running at lower speed (default)
  spidat |=  0<<6;   // RESET: 00 = Neither ADC is in Reset mode (default)
  spidat |=  0<<4;   // SHUTDOWN: 00 = Neither channel is in shutdown (default)
  spidat |=  0<<2;   // VREFEXT: 0 = Internal voltage reference enabled (default)
  spidat |=  0<<1;   // CLKEXT: 0 = crystal resonator connected 1 = External clock drive by MCU on OSC1 pin
  send_MCP3911(MCP3911_CONFIG_ADDR, spidat);  

  // ADC Calibration
  spidat = (caliblob->ADC_OCAL0)<<4;  // shift <<4 for compliance with 20bit result readings
  send_MCP3911(MCP3911_OFFCAL0_ADDR, spidat);  
  spidat = (caliblob->ADC_OCAL1)<<4;
  send_MCP3911(MCP3911_OFFCAL1_ADDR, spidat);  
  spidat = caliblob->ADC_GCAL0;
  send_MCP3911(MCP3911_GAINCAL0_ADDR, spidat);  
  spidat = caliblob->ADC_GCAL1;
  send_MCP3911(MCP3911_GAINCAL1_ADDR, spidat);  

  vTaskDelay(2);  // wait for all transfers to be ended

  // prepare SPI for continiously reding ADC data on GPIO interrupt generated by MCP3911_DR signal  
  // using low level -> using high level library functions isn't allowed any more
  // so don't use spi_device_transmit() after this point
  host->user.usr_mosi_highpart = 0;
  host->user.usr_mosi = 1;  // We have to send some dummy data
  host->user.usr_miso = 1;  // We have to receive some data
  host->mosi_dlen.usr_mosi_dbitlen = 48-1;    // Set mosi dbitlen
  host->miso_dlen.usr_miso_dbitlen = 48-1;    // Set miso dbitlen
  host->addr= 0x01000000;   // set address for reading ADC channel 0 --- byte little endianness (address = 0 | R/W = 1)
}

//############################################################################
void stop_MCP3911() {
  uint8_t CONF  = 0x34; // SHUTDOWN bits 
  send_MCP3911(MCP3911_CONFIG_ADDR, CONF);
}

//############################################################################
void send_MCP3911(uint8_t address, uint32_t spidat) {  
  // High byte of spidat goes to high byte in MCP3911 register.
  MCP3911_transmit.addr = address;
  switch(address) {
    case MCP3911_MOD_ADDR:
    case MCP3911_GAIN_ADDR:
      MCP3911_transmit.length   = 8;
      MCP3911_transmit.rxlength = 8;
      MCP3911_transmit.tx_data[0] = spidat;
      break;
    case MCP3911_PHASE_ADDR:
    case MCP3911_STATUS_ADDR:
    case MCP3911_CONFIG_ADDR:
      MCP3911_transmit.length   = 16;
      MCP3911_transmit.rxlength = 16;
      MCP3911_transmit.tx_data[0] = spidat>>8;  // send high byte first
      MCP3911_transmit.tx_data[1] = spidat;
      break;
    case MCP3911_OFFCAL0_ADDR:
    case MCP3911_GAINCAL0_ADDR:
    case MCP3911_OFFCAL1_ADDR:
    case MCP3911_GAINCAL1_ADDR:
      MCP3911_transmit.length   = 24;
      MCP3911_transmit.rxlength = 24;
      MCP3911_transmit.tx_data[0] = spidat>>16;  // send high byte first
      MCP3911_transmit.tx_data[1] = spidat>>8;
      MCP3911_transmit.tx_data[2] = spidat;
      break;
    default:
      //send_UART0(s, sprintf(s, "ERR MCP3911 address %X\n\r", address));
      break;

  }
  esp_err_t ret = spi_device_transmit(MCP3911_dev, &MCP3911_transmit);
  //if(ESP_OK != ret) { send_UART0(char s[32], sprintf(s, "ERR SPI send %X\n\r", ret));}
}

//############################################################################
int16_t read_MCP3911(uint8_t address) { // ADC is used in 16 bit mode
  esp_err_t ret;
  MCP3911_transmit.tx_data[0] = 0;
  MCP3911_transmit.tx_data[1] = 0;
  MCP3911_transmit.tx_data[2] = 0;
  
  if((MCP3911_MOD_ADDR == address) | (MCP3911_GAIN_ADDR == address)) {
      MCP3911_transmit.length   = 8;
      MCP3911_transmit.rxlength = 8;
  } else {
      MCP3911_transmit.length   = 16;
      MCP3911_transmit.rxlength = 16;
  }

  MCP3911_transmit.addr = address | 0x01; // set read bit in address byte
  ret = spi_device_transmit(MCP3911_dev, &MCP3911_transmit);
  //if(ESP_OK != ret) { uart_write_bytes(UART_NUM_0, err, sprintf(err, "ERR SPI send %X\n\r", ret));}
  return (MCP3911_transmit.rx_data[0]<<8) + MCP3911_transmit.rx_data[1];
}

/*
static void IRAM_ATTR MCP3911_DR_isr_handler(void* arg)  {  // GPIO interrupt on DataReady signal from MCP3911
  static volatile IRAM_ATTR uint32_t SPI_buf[2][MCP3911_buf_length];  // SPI data in big endian byte order
  static volatile IRAM_ATTR uint32_t * bufptr;
  static IRAM_ATTR uint32_t bufidx = 0; 
  static IRAM_ATTR int32_t  pingpong = 0;
  spi_dev_t *host = (spi_dev_t *)REG_SPI_BASE(2); // = SPI2->host;

  SPI_buf[pingpong][bufidx] = host->data_buf[0];

  host->data_buf[0] = 0; // keep MOSI data line quite
  
  host->cmd.usr=1;  // start the transfer
  //host->user.usr_miso = 0; // the purpose of this setting is not clear
    
  bufidx++;
  if(MCP3911_buf_length <= bufidx) { 
    bufidx = 0;
    bufptr = SPI_buf[pingpong];
    xQueueSendFromISR(GPIO_evt_queue, &bufptr, NULL);
    pingpong ^= 0x00000001; 
  }
}
*/

//############################################################################
//#
void IRAM_ATTR MCP3911_DR_isr_handler(void* arg)  {  // GPIO interrupt on DataReady signal from MCP3911
  static volatile IRAM_ATTR rxbuffer_t SPI_buf[2][MCP3911_buf_length];  // SPI data in big endian byte order
  static volatile IRAM_ATTR uint64_t * bufptr;
  static IRAM_ATTR uint32_t bufidx = 0; 
  static IRAM_ATTR int32_t  pingpong = 0;
  // byte access on IRAM_ATTR data stalls the CPU !
  
  SPI_buf[pingpong][bufidx].i32[0] = host->data_buf[0];
  SPI_buf[pingpong][bufidx].i32[1] = host->data_buf[1];
  
  host->data_buf[0] = 0; // keep MOSI data line quiet
  host->data_buf[1] = 0; // keep MOSI data line quiet
  
  host->cmd.usr = 1;  // start the transfer for reading the next results
  //host->user.usr_miso = 0; // the purpose of this setting is not clear
    
  bufidx++;
  if(MCP3911_buf_length <= bufidx) { 
    bufidx = 0;
    bufptr = SPI_buf[pingpong];
    xQueueSendFromISR(GPIO_evt_queue, &bufptr, NULL);
    pingpong ^= 0x00000001; 
  }
}

//############################################################################
//############################################################################
//############################################################################
//############################################################################
