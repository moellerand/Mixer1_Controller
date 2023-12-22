/*  UART.c
 *  by Andi Moeller
 *  20231122
 *  20231212
 * 
 * 
 * 
 */

//############################################################################
//############################################################################

#include "UART.h"
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <soc/reg_base.h>
#include <soc/soc.h>
#include "ESP32_Controller.h"
#include "ESP32_Terminal.h"

//static QueueHandle_t UART0_queue;  // Setup UART buffered IO with event queue
//static QueueHandle_t UART1_queue;  // Setup UART buffered IO with event queue

#define UART1_TxD     27    //     
#define UART1_RxD     39    // 
#define UART1_TxE     22    //
#define UART_BUF_SIZE             (1024)  // buffer size


//############################################################################
TaskHandle_t  UART1_event = NULL;

//############################################################################
//#
void init_UART0() {   // terminal interface
  esp_log_level_set("uart_events", ESP_LOG_INFO); // ?????? Set UART log level
  TaskHandle_t  UART0_event = NULL;

  // Configure UART parameters
  uart_config_t UART_config = {
    .baud_rate =    MONITOR_UART_BAUD_RATE,
    .data_bits =    UART_DATA_8_BITS,
    .parity =       UART_PARITY_DISABLE,
    .stop_bits =    UART_STOP_BITS_1,
    .flow_ctrl =    UART_HW_FLOWCTRL_DISABLE,
    //.source_clk = UART_SCLK_APB,
  };
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &UART_config));

  //Install UART driver, and get the queue. The Queue won't be used.
  //uart_driver_install(uart_port_t UART_NUM_0, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, NULL, ESP_INTR_FLAG_IRAM));
  ESP_ERROR_CHECK(uart_set_mode(UART_NUM_0, UART_MODE_UART)); //UART_MODE_RS485_HALF_DUPLEX

  // Set UART0 pins(TX: IO1, RX: IO3, RTS: IO18, CTS: IO19)
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3,-1,-1));

  //Create a task to handler UART event from ISR
  xTaskCreate(UART0_event_task, "UART0_event_task", 2048, NULL, 3, &UART0_event);
}

//############################################################################
//#
void init_UART1() {   // used for controlling a brushless motor driver (project Mixer1)
  // Configure UART parameters
  uart_config_t UART_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_DEFAULT,
  };

  //Install UART driver, and get the queue. The Queue won't be used.
  //uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
  uart_driver_install(UART_NUM_1, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, NULL, ESP_INTR_FLAG_IRAM);
  uart_param_config(UART_NUM_1, &UART_config);

  // Set UART1 pins(TX: IO27, RX: IO39, RTS: 22, CTS: x)
  uart_set_pin(UART_NUM_1, UART1_TxD, UART1_RxD, UART1_TxE, -1);
  uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX); //UART_MODE_UART
  
  //Create a task to handler UART event from ISR
  //ESP_ERROR_CHECK(xTaskCreate(UART1_event_task, "UART1_event_task", 2048, NULL, 2, &UART1_event));
  xTaskCreate(UART1_event_task, "UART1_event_task", 2048, NULL, 2, &UART1_event);
}

//############################################################################
//#
void send_UART0(char* s, size_t len) {
  uart_write_bytes(UART_NUM_0, s, len);
}

//############################################################################
//#
void UART0_event_task(void * pvParameters) {
    static int8_t  rxc;  // received character
    static int32_t rxi = 0;  // number of bytes received in the command line
    static int8_t command[256];
    
    for(;;) { // waiting for UART event.
      uart_read_bytes(UART_NUM_0, &rxc, 1, portMAX_DELAY); // read one by one
      switch(rxc) { 
        case '\n':
          com_parser(command, rxi);
          rxi = 0;
          break;
        case '\r':  // strip cr
          break;
        default:    // store character in command line
          command[rxi] = rxc;
          if(rxi < (sizeof(command) - 1)) rxi++; 
          break;
      }
    }
}

//############################################################################
//#
void send_UART1(char* s, size_t len) {
  uart_write_bytes(UART_NUM_1, s, len);
}

//############################################################################
//#
void UART1_event_task(void * pvParameters) {
  //static int32_t rxi = 0;  // number of bytes received in the command line
  //char  rxc;  // received character
  static char  rxb[256];  // buffer
    
  for(;;) { // waiting for UART event.
    /*
    uart_read_bytes(UART_NUM_1, &rxc, 1, portMAX_DELAY);// // read one by one
    sprintf(rxb, "%02X.", rxc);
    send_UART0((char *)rxb, 3);
    */
    uart_read_bytes(UART_NUM_1, rxb, 7, portMAX_DELAY); // / portTICK_PERIOD_MS);// // read seven

    if((0 == strncmp(rxb, "\x01\x03\x02", 3))) {
      send_UART0(rxb, sprintf(rxb, "RPM = %3d\r\n", 5 * rxb[3]));
    } else { 
      //sprintf(rxb+16, "%02X.%02X.%02X.%02X\r\n", rxb[0], rxb[1], rxb[2], rxb[3]);
      //send_UART0(rxb+16, 13);
      vTaskDelay(1);
      uart_flush(UART_NUM_1);
    }
   
    /*
    switch(rxc) { 
      case '\n':
        send_UART0(rxb, 10);
        rxi = 0;
        break;
      default:    // store character in command line
        rxb[rxi] = rxc;
        //if(rxi < (sizeof(rxb) - 1)) rxi++; 
        break;
    }
    */
  }
}

//############################################################################
//#
//############################################################################
//############################################################################
//############################################################################
//############################################################################


