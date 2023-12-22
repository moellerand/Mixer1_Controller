/*  ESP32_Controller0
 *  A simple application for controlling a BLDC motor'
 *  by Andi Moeller
 *  20231122
 *  20231209
 * 
 * 
 * 
 */


//############################################################################
//############################################################################
// custom header files
#include "ESP32_Controller.h"
#include "UART.h"
#include "FLASH.h"
#include "TMP75.h"
#include "MCP3911.h"
#include "ITS90_Data.h"
#include "BLD510S.h"
#include "ESP32_Terminal.h"
//#include "ILI9341.h"

//############################################################################


// globals
esp_err_t ret;  //
char err[128];  //

extern ADC_Result_t ADC_Result;
extern MCP3911_calib_data_t * ADC_CalibData;
float   TEMP_ColdJunction;  // °C

//############################################################################
//############################################################################
//#
float ADCtoVolt(int32_t adc) { 
  #define Vref  (2.0 * 0.600)   // Volt
  #define ADC_MAXVAL 0x080000l   // ADC full range output (20bit)
  const float A2V = Vref / (1.5 * 2.0 * ADC_MAXVAL); // Vref[V] / (1.5 * G(=2) * 2²³>>4)
  return(adc * A2V); 
}

//############################################################################
//#
float ADCtoTemperature(int32_t adc) {   // approximation for cold juction temperatures near to 20°C
#include "ITS90_Data.h"
  float CJ_Volt; 
  CJ_Volt = Volt_20C + Volt_20C_divC * (TEMP_ColdJunction - 20.0); // voltage at the cold junction with respect to  0°C [V]
  float Volt;
  Volt = ADCtoVolt(adc);  // voltage calculated from ADC value
  Volt += CJ_Volt;        // add calculated cold junction voltage to measured voltage
  int32_t i;
  float Coef, Temp;
  for(i = 1; i < ITS90_MAX_POINTS; i++) {
    if(ITX90_table[i].Volt >= Volt) break;
  }
  Coef = (ITX90_table[i].Temp - ITX90_table[i-1].Temp) / (ITX90_table[i].Volt - ITX90_table[i-1].Volt);
  Temp =  ITX90_table[i].Temp - (ITX90_table[i].Volt - Volt) * Coef;
  return(Temp);
}

//############################################################################
//#
void vTaskTogLED( void * pvParameters ) {
  // Block for 500ms.
  const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
  int i = 0;
  for( ;; ) {
      // Simply toggle the LED every 500ms, blocking between each toggle.
      i ^= 0x1;
      if(i) {
        gpio_set_level(PIN_OUT0, 1);
        gpio_set_level(PIN_OUT2, 0);
      } else {
        gpio_set_level(PIN_OUT0, 0);
        gpio_set_level(PIN_OUT2, 1);
      } 
      vTaskDelay( xDelay ); 
      TEMP_ColdJunction = read_TMP175();
  }
}

//############################################################################
//############################################################################
//#
void app_main() {
  setup();
  int i = 0;
  for(;;) {
    //uart_write_bytes(UART_NUM_0, err, sprintf(err, "\t%+5d\n\r", i));
    i++;
    vTaskDelay(5);
    if(i & 0x1) gpio_set_level(PIN_OUT1, 1); else gpio_set_level(PIN_OUT1, 0);
  }
}

//############################################################################
//#
void setup() { 
  // configure pins 12,13,14,15 in GPIO mode (not as JTAG)
  //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[12], PIN_FUNC_GPIO);
  //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[13], PIN_FUNC_GPIO);
  //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[14], PIN_FUNC_GPIO);
  //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[15], PIN_FUNC_GPIO);
  // setup GPIOs for OUTx pins
  gpio_set_direction(PIN_OUT0, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_OUT1, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_OUT2, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_OUT3, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_OUT4, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_OUT5, GPIO_MODE_OUTPUT);

  init_UART0();  // Monitor
  init_UART1();  // RS485 MODBUS interface to BLDC motor controller
  
  init_I2C();     //
  init_TMP175();  //

  init_NVS();     // 
  read_flash_blob(&ADC_CalibData); // pass pointer to pointer and read data from flash (NVS)

  init_SPI2();    // SPI 2 connects to the MCP3911 using the dedicated io-mux pins
  init_MCP3911(ADC_CalibData); // configure ADC
  init_Interrupt_on_MCP3911_DR(); // install GPIO intererupt on ADC data ready

  //init_SPI3();
  //init_ILI9341();


  xTaskCreate(vTaskTogLED, "vTaskTogLED", 2048, NULL, 2, NULL);
  // not used so far
  //init_Timer();  

  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT, // ADC_BITWIDTH_ (9..13) ?
    .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
}

//############################################################################
//############################################################################

//############################################################################