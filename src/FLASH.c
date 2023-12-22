/*  FLASH.c
 *  by Andi Moeller
 *  20231209
 *
 * 
 * 
 * 
 */

// includes

#include "ESP32_Controller.h"
#include "FLASH.h"
#include <nvs_flash.h>
#include <nvs.h>

// globals
static nvs_handle_t CalibHandle;

//############################################################################
//#
void init_NVS() { // init non volatile storage on flash memory partition
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) printf("Init NVS Error 1: %s\n", esp_err_to_name(err));  //????????
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
      if (err != ESP_OK) printf("Init NVS Error 2: %s\n", esp_err_to_name(err));  //????????
  }
  ESP_ERROR_CHECK( err );
}

//############################################################################
//#
void read_flash_blob(MCP3911_calib_data_t** cali_ptr) { 
  static volatile MCP3911_calib_data_t cal_data;
  *cali_ptr = &cal_data; // pointer value to cal_data is passed to the calling function
  esp_err_t err;
  size_t blobsize = sizeof(MCP3911_calib_data_t);
  
  err = nvs_open("Calib_Data", NVS_READWRITE, &CalibHandle); // Open NVS partition
  if (err != ESP_OK) printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  // Get data blob size and read the blob
  err = nvs_get_blob(CalibHandle, "caliblob", NULL, &blobsize); // don't try to get the blob if it is not yet existant 
  if (err != ESP_OK) printf("Read NVS Error 1: %s\n", esp_err_to_name(err));
  if (err == ESP_OK) {
    err = nvs_get_blob(CalibHandle, "caliblob", (void *)&cal_data, &blobsize);
    if (err != ESP_OK) printf("Read NVS Error 2: %s\n", esp_err_to_name(err));
  } else {
    printf("Read NVS Error 3: No caliblob found.\n");
  }
  
  nvs_close(CalibHandle);  // Close
}

//############################################################################
//#
void write_flash_blob(MCP3911_calib_data_t* caliblob) { 
  esp_err_t err;
  //size_t blobsize = 0;
  
  err = nvs_open("Calib_Data", NVS_READWRITE, &CalibHandle); // Open NVS partition
  if (err != ESP_OK) printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  
  /*// Get data blob size and read the blob
  err = nvs_get_blob(CalibHandle, "caliblob", NULL, &blobsize);   // by pasing the NULL pointer, blobsize will be read
  if (err != ESP_OK) printf("Read1 NVS Error: %s\n", esp_err_to_name(err)); // don't try to get the blob if it is not yet existant 
  if (err == ESP_OK) {
    err = nvs_get_blob(CalibHandle, "caliblob", caliblob, &blobsize); // read the blob
    if (err != ESP_OK) printf("Read2 NVS Error: %s\n", esp_err_to_name(err));
  } else {
    printf("Read1 NVS Error: No caliblob found.\n");
  }
  */
  err = nvs_set_blob(CalibHandle, "caliblob", caliblob, sizeof(MCP3911_calib_data_t));  // Write new value to nvs
  if (err != ESP_OK) printf("Write NVS Error: %s\n", esp_err_to_name(err));
  
  err = nvs_commit(CalibHandle); // Commit write
  if (err != ESP_OK) printf("Commit NVS Error: %s\n", esp_err_to_name(err));

  nvs_close(CalibHandle);  // Close
}

//############################################################################
//############################################################################
//############################################################################
//############################################################################
//############################################################################



