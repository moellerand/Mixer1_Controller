/*  TMP75.h
 *  Header file
 *  by Andi Moeller
 *  20231209
 *
 *  
 * 
 * 
 */

//############################################################################


#ifndef TMP75
#define TMP75
//############################################################################

#include <driver/i2c.h>


void init_I2C();
void init_TMP175();
float read_TMP175();


//############################################################################
#endif  // ifndef TMP75

//############################################################################
//############################################################################
//############################################################################
