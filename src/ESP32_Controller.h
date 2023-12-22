/*  ESP32_Controller.h
 *  Header file
 *  by Andi Moeller
 *  20231206
 *
 * 
 * 
 * 
 */

//############################################################################


#ifndef ESP_Controller
#define ESP_Controller
//############################################################################
#define REVISION     "MIXER1 R0.1"
//const char REVISION[] = "MIXER1 R0.1";

#include "esp_chip_info.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <driver/ledc.h>
#include <esp_adc/adc_oneshot.h> //<driver/adc.h>
//#include <driver/i2c_master.h>  // not yet available 7 coming soon
#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_intr_alloc.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include <esp_pm.h>     // power management
#include <esp_log.h>
#include <sdkconfig.h>


//############################################################################
// fine defines

#define PIN_OUT0  4
#define PIN_OUT1 16
#define PIN_OUT2 17
#define PIN_OUT3 21
#define PIN_OUT4 22
#define PIN_OUT5 27

#define PIN_SPI2_MISO 12    // SPI2 Data Input 
#define PIN_SPI2_MOSI 13    // SPI2 Data Input 
#define PIN_SPI2_SCK  14    // SPI2 SCK
#define PIN_SPI2_NCS  15    // SPI NCS (/CS) Chip Select Output

#define PIN_I2C_SDA   33    // I2C 
#define PIN_I2C_SCL   32    // 

#define PIN_ADC_DR    35    // ADC data ready signal

//############################################################################
// type definitions

//############################################################################
//#
// prototypes
void setup();   // setup peripherals
void init_Timer();
float ADCtoVolt(int32_t adc);
float ADCtoTemperature(int32_t adc);
void vTaskTogLED(void * pvParameters);

//############################################################################
//#
// global variables used in RTOS tasks

//############################################################################
#endif  // ifndef ESP_Controller
//############################################################################
//############################################################################
//############################################################################


