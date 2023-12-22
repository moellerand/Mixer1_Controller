/*  TMP75.c
 *  Reads TMP75 digital temperature sensor via I2C
 *  by Andi Moeller
 *  20231209
 *
 *  
 * 
 * 
 */

//############################################################################
#include "ESP32_Controller.h"
#include "TMP75.h"

//############################################################################
//#
void init_I2C() {
  i2c_config_t i2c_master_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = PIN_I2C_SDA,
    .scl_io_num = PIN_I2C_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
  };

  #define I2C_MASTER_NUM  0
  #define I2C_MASTER_TX_BUF_DISABLE   0    
  #define I2C_MASTER_RX_BUF_DISABLE   0    

  i2c_param_config(I2C_MASTER_NUM, &i2c_master_config);
  i2c_driver_install(I2C_MASTER_NUM, i2c_master_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, (ESP_INTR_FLAG_IRAM));
}

//############################################################################
//#
void init_TMP175() {
  #define TMP175_ADDR 0x48
  uint8_t write_buf[3];
  write_buf[0] = 0x01;    // pointer to configuration register
  write_buf[1] = 0x03<<5; // resolution 12 bits -> 0,0625°C
  write_buf[2] = 0x03<<5; // resolution 12 bits -> 0,0625°C
  i2c_master_write_to_device(I2C_MASTER_NUM, TMP175_ADDR, write_buf, 3, 20 / portTICK_PERIOD_MS);
}

//############################################################################
//#
float read_TMP175() { // temperature reading is 1/16 °C
  //uint8_t reg_addr, uint8_t *data, size_t len
  uint8_t write_buf[2] = {0x00, TMP175_ADDR};
  uint16_t Celsius = 0;  // centi Celsius
  i2c_master_write_read_device(I2C_MASTER_NUM, TMP175_ADDR, write_buf, 2, (uint8_t *)&Celsius, 2, 20 / portTICK_PERIOD_MS);
  Celsius>>=4;  // sign extension
  return (Celsius * (1.0/16.0));  
}



//############################################################################



//############################################################################


//############################################################################
//############################################################################
//############################################################################
