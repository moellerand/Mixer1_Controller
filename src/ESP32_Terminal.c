/*  ESP32_Terminal.c
 *  UART Terminal
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
#include "ESP32_Terminal.h"
#include "FLASH.h"
#include "UART.h"
#include "BLD510S.h"
#include "MCP3911.h"


//############################################################################
// 
typedef union {
    double_t  d;
    int32_t   l;
    int32_t   f;
    int16_t   i[4]; 
    int8_t    c[8];
} parameter_t;

typedef struct { 
    int16_t     instruct;
    parameter_t par; 
} command_t;

extern ADC_Result_t ADC_Result;  //
extern MCP3911_calib_data_t * ADC_CalibData;

extern float TEMP_ColdJunction; //

//############################################################################
// prototypes
void execute(command_t comm);

//############################################################################
//#
enum {no, rr, test, getr, getz, ga, gf, gg, gr, gu, gett, led, mcp, sig, off, minit, mspeed, mread, mg, adc_ocal0, adc_ocal1, adc_gcal0, adc_gcal1, adc_cal_show, adc_cal_store};

void com_parser(int8_t * cs, int32_t length) {
  static command_t comm;
  comm.par.d = 0.0;
  comm.instruct = -1;
  int8_t * param_end; // dummy for strtod()
  cs[length] = '\0';  // string end for strtod()
  int32_t i = 0;

  // look for the instruction
  do {  // strip leading delimiters
    if(!(('\t' == cs[i]) | (' ' == cs[i]))) break;
    i++;
  } while(i < length);

  comm.instruct = no; // default 
  // compare to implemented instructions
  switch(cs[i]) { // switch() as a preselection because it is faster than strncmp() 
    case 'a':
      if(0 == strncmp((char *)&cs[i], "adc_ocal0", 9)) {comm.instruct = adc_ocal0; break; }
      if(0 == strncmp((char *)&cs[i], "adc_ocal1", 9)) {comm.instruct = adc_ocal1; break; }
      if(0 == strncmp((char *)&cs[i], "adc_gcal0", 9)) {comm.instruct = adc_gcal0; break; }
      if(0 == strncmp((char *)&cs[i], "adc_gcal1", 9)) {comm.instruct = adc_gcal1; break; }
      if(0 == strncmp((char *)&cs[i], "adc_cal_show", 10)) {comm.instruct = adc_cal_show; break; }
      if(0 == strncmp((char *)&cs[i], "adc_cal_store", 10)) {comm.instruct = adc_cal_store; break; }
      break;
    case 'g':
      if(0 == strncmp((char *)&cs[i], "ga", 2)) {comm.instruct = ga; break; }
      if(0 == strncmp((char *)&cs[i], "gf", 2)) {comm.instruct = gf; break; }
      if(0 == strncmp((char *)&cs[i], "gg", 2)) {comm.instruct = gg; break; }
      if(0 == strncmp((char *)&cs[i], "gr", 2)) {comm.instruct = gr; break; }
      if(0 == strncmp((char *)&cs[i], "gu", 2)) {comm.instruct = gu; break; }
      if(0 == strncmp((char *)&cs[i], "gett", 4)) {comm.instruct = gett; break; }
      if(0 == strncmp((char *)&cs[i], "getz", 4)) {comm.instruct = getz; break; } 
      if(0 == strncmp((char *)&cs[i], "getr", 4)) {comm.instruct = getr; break; } 
      break;
    case 'l': if(0 == strncmp((char *)&cs[i], "led", 3))  comm.instruct = led; break;
    case 'm': 
      if(0 == strncmp((char *)&cs[i], "minit", 2))  {comm.instruct = minit; break; }
      if(0 == strncmp((char *)&cs[i], "mspeed", 2)) {comm.instruct = mspeed; break; }
      if(0 == strncmp((char *)&cs[i], "mread", 2)) {comm.instruct = mread; break; }
      if(0 == strncmp((char *)&cs[i], "mg", 2)) {comm.instruct = mg; break; }
      break;
    case 'o': if(0 == strncmp((char *)&cs[i], "off", 3))  comm.instruct = off; break;
    case 'r': if(0 == strncmp((char *)&cs[i], "rr", 2))  comm.instruct = rr; break;
    case 't': if(0 == strncmp((char *)&cs[i], "test", 4))  comm.instruct = test; break;
    default:
      comm.instruct = no; 
      strncpy((char *)comm.par.c, (char *)&cs[i], 4);
      break;
  }
  
  // look for a delimiter, a sign or a decimal dot
  for(i = i; i < length; i++) { // 
    if(('\t' == cs[i]) | (' ' == cs[i]) | ('+' == cs[i]) | ('-' == cs[i]) | ('.' == cs[i])) break;
  }
  
  // look for a number
  for(i = i; i < length; i++) { 
    if((('0' <= cs[i]) & (cs[i] <= '9')) | ('+' == cs[i]) | ('-' == cs[i]) | ('.' == cs[i])) {  // string contains a number
      comm.par.d = strtod((char *)&cs[i], (char **)&param_end);  // grap the number
      break;
    }
  }
  execute(comm);
}  // end of com_parser()

//############################################################################
//#
void execute(command_t comm) { 
  static char s[128];
  switch(comm.instruct) {
    case rr:  // reset ESP32
      send_UART0(s, sprintf(s, "ESP32 restarts..\n\r"));
      esp_restart();
      break;
    case ga:  // get averaged ADC data
      send_UART0(s, sprintf(s, "%+7ld\t%+7ld\n\r", ADC_Result.AV[0], ADC_Result.AV[1]));
      break;
    case gf:  // get ADC filtered data
      send_UART0(s, sprintf(s, "%+7ld\t%+7ld\n\r", ADC_Result.F1[0], ADC_Result.F1[1]));
      break;
    case gg:  // get ADC filtered data
      send_UART0(s, sprintf(s, "%+7ld\t%+7ld\n\r", ADC_Result.F2[0], ADC_Result.F2[1]));
      break;
    case gr:  // get ADC raw data
      send_UART0(s, sprintf(s, "%+7ld\t%+7ld\n\r", ADC_Result.raw[0], ADC_Result.raw[1]));
      break;
    case gu:  // get ADC voltage [µV]
      send_UART0(s, sprintf(s, "%+.6f\t%+.6f\n\r", ADCtoVolt(ADC_Result.F1[0]), ADCtoVolt(ADC_Result.F1[1])));
      break;
    case getr:  // get firmware revision
      send_UART0(s, sprintf(s, "FW Revison: %s\n\r", REVISION));
      break;
    case gett:  // get TMP175 temperature
      send_UART0(s, sprintf(s, "%+03.2f°C\t%+03.2f°C\t%+03.2f°C\n\r", TEMP_ColdJunction, ADCtoTemperature(ADC_Result.F1[0]), ADCtoTemperature(ADC_Result.F1[1])));
      break;
    case led: // ?????? switch red LED on and off
      //if (0 == comm.par.d) gpio_set_level(PIN_LED_RED, 0); 
      //else gpio_set_level(PIN_LED_RED, 1);
      break; 
    case test:    // test number recognition
      send_UART0(s, sprintf(s, "Test Number = %1.16lE \n" , comm.par.d));
      break;
    case adc_ocal0:    // Set ADC channel0 offset for storing in NVS
      ADC_CalibData->ADC_OCAL0 = (int32_t)comm.par.d;
      send_UART0(s, sprintf(s, "Set ADC channel 0 offset: %-ld\n" , ADC_CalibData->ADC_OCAL0));
      break;
    case adc_ocal1:    // Set ADC channel1 offset for storing in NVS
      ADC_CalibData->ADC_OCAL1 = (int32_t)comm.par.d;
      send_UART0(s, sprintf(s, "Set ADC channel 1 offset: %-ld\n" , ADC_CalibData->ADC_OCAL1));
      break;
    case adc_gcal0:    // Set ADC channel0 gain for storing in NVS
      ADC_CalibData->ADC_GCAL0 = (int32_t)comm.par.d;
      send_UART0(s, sprintf(s, "Set ADC channel 0 gain: %ld\n" , ADC_CalibData->ADC_GCAL0));
      break;
    case adc_gcal1:    // Set ADC channel1 gain for storing in NVS
      ADC_CalibData->ADC_GCAL1 = (int32_t)comm.par.d;
      send_UART0(s, sprintf(s, "Set ADC channel 1 gain: %ld\n" , ADC_CalibData->ADC_GCAL1));
      break;
    case adc_cal_show:    // show calibration data
      send_UART0(s, sprintf(s, "OCAL0:%+6ld\tOCAL1:%+6ld\tGCAL0:%+6ld\tGCAL1:%+6ld\n", ADC_CalibData->ADC_OCAL0, ADC_CalibData->ADC_OCAL1, ADC_CalibData->ADC_GCAL0, ADC_CalibData->ADC_GCAL1));
      break;
    case adc_cal_store:    // write calibration data to non volatile storage (nvs)
      // calibration as of 20231210 OCAL0:  -199    OCAL1:  -198    GCAL0:-147900   GCAL1:-125500
      write_flash_blob(ADC_CalibData);
      break;
    case minit:    // 
      BLD510S_init();
      break;
    case mspeed:  // 
      int32_t rpm = comm.par.d;     //
      BLD510S_set_speed(rpm);       // 
      break;
    case mread:
      BLD510S_read_speed();
      break;
    case mg:
      BLD510S_read_status();
      break;
    case no:
    default:      // unknown command
      send_UART0(s, sprintf(s, "Unknown Command: %.4s\t  Parameter: %lf \n" , (const char*)comm.par.c, comm.par.d));
      break;
  }
} // end of execute()




//############################################################################
//############################################################################
//############################################################################
//############################################################################
