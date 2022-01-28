/*
 * File:   debounce.c
 * Author: root
 *
 * Created on June 22, 2021, 10:57 AM
 */


#include <xc.h>
#include "debounce.h"


bool checkPushButton(bool pushButtonPin)
{
    //Almacenar la lectura del pin
    bool button_state = pushButtonPin;
    
    //Checar si el boton esta presionado
    if(button_state == false && button_state != pushButtonStatus)
    {
        //Actualizar buttonState;
        pushButtonStatus = button_state;
        //Delay antirebote (debouncing)
        __delay_ms(20);
        return false;
    }
    
    //Checar si el boton ha sido liberado
    if(button_state == true && button_state != pushButtonStatus)
    {
        //Actualizar boton como liberado
        pushButtonStatus = button_state;
        __delay_ms(20);
        return true;
    }
    
    //Si ninguna de la condiciones se cumple retornar
    return false;
}