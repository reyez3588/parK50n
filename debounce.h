/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __DEBOUNCE_H
#define	__DEBOUNCE_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include "mcc_generated_files/device_config.h" //para utilizar __delay_ms()
#include "../c99/stdbool.h" // para variables tipo bool


//Variable to track pushbutton status
bool pushButtonStatus = true;

//Funcion para revisar el estado del boton
// Retorna "true" cuando el boton se libera, en caso contrario "false"
bool checkPushButton(bool pushButtonPin);


#endif	/* XC_HEADER_TEMPLATE_H */

