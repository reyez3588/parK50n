/*
 * File:   xlcd.c
 * Author: Jesus Reyes Gomez
 *
 * Created on 21 de mayo de 2021, 04:51 PM
 */


#include "xlcd.h"
#include "mcc_generated_files/device_config.h" //necesaria para Delays
#include "mcc_generated_files/examples/i2c1_master_example.h" //necesaria para I2C
#include "mcc_generated_files/pin_manager.h" 


//Funcion para inicializar el LCD
void initLCD(void)
{
    OpenXLCD(FOUR_BIT & LINES_5X7); //para LCD 16x2 y 20x4
    WriteCmdXLCD( DON & CURSOR_OFF & BLINK_OFF); // despues de comando Entry mode set se requiere un retardo de 2ms
    __delay_ms(2);
    return;
}

/* Funcion para hacer toggle a luz de fondo del display
 se puede utilizar en cualquier momento*/
void toggleBacklight(void)
{
   
//    I2C1_ReadNBytes(LCDaddress, &dataIn, 1);
//    //NOP();
//    stateLight= dataIn & 0b00001000;
//    //NOP();
//    
    if(stateLight == 0b00001000) //si esta prendido apagarlo
    {
        backLightOFF();
    }
    else if (stateLight == 0b00000000)
    {
        backLightON();
    }
    return;
}

/*Funcion para apagar la luz de fondo del LCD*/
void backLightOFF(void)
{
    stateLight = 0b00000000;
    I2C1_WriteNBytes(LCDaddress, &stateLight, 1);
    return;
}

/*Funcion para activar la luz de fondo del display, por default despues del 
 reset la luz esta activada*/
void backLightON(void)
{
    stateLight = 0b00001000;
    I2C1_WriteNBytes(LCDaddress, &stateLight, 1);
    return;
}



/********************************************************************
*       Function Name:  BusyXLCD                                    *
*       Return Value:   char: busy status of LCD controller         *
*       Parameters:     void                                        *
*       Description:    This routine reads the busy status of the   *
*                       Hitachi HD44780 LCD controller.             *
********************************************************************/
unsigned char BusyXLCD(void)
{
    /*Esta funcion no esta implementada
     se puede realizar pero habria que realizar la lectura del LCD*/
    return 1;
}

/********************************************************************
*       Function Name:  OpenXLCD                                    *
*       Return Value:   void                                        *
*       Parameters:     lcdtype: sets the type of LCD (lines)       *
*       Description:    This routine configures the LCD. Based on   *
*                       the Hitachi HD44780 LCD controller. The     *
*                       routine will configure the I/O pins of the  *
*                       microcontroller, setup the LCD for 4- or    *
*                       8-bit mode and clear the display.           *
********************************************************************/
void OpenXLCD(unsigned char lcdtype)
{
        // Delay for 15ms to allow for LCD Power on reset
    __delay_ms(15);
 //-------------------reset procedure through software----------------------       
    WriteCmdXLCD(0x30);
    __delay_ms(5);
         

    WriteCmdXLCD(0x30);
    __delay_ms(1);



    WriteCmdXLCD(0x32);
	__delay_ms(1);
//------------------------------------------------------------------------------------------


        // Set data interface width, # lines, font
        WriteCmdXLCD(lcdtype);          // Function set cmd
        __delay_ms(1);              // Wait if LCD busy

        // Turn the display on then off
        WriteCmdXLCD(DOFF&CURSOR_OFF&BLINK_OFF);        // Display OFF/Blink OFF
        __delay_ms(1);              // Wait if LCD busy
        WriteCmdXLCD(DON&CURSOR_ON&BLINK_ON);           // Display ON/Blink ON
        __delay_ms(1);              // Wait if LCD busy

        // Clear display
        WriteCmdXLCD(0x01);             // Clear display
        __delay_ms(1);              // Wait if LCD busy

        // Set entry mode inc
        WriteCmdXLCD(SHIFT_DISP_LEFT);   // Entry Mode
        __delay_ms(2);              // Wait if LCD busy

        // Set DD Ram address to 0
        SetDDRamAddr(0x00);                // Set Display data ram address to 0
         __delay_ms(1);              // Wait if LCD busy

        return;
}

/********************************************************************
*       Function Name:  putrsXLCD
*       Return Value:   void
*       Parameters:     buffer: pointer to string
*       Description:    This routine writes a string of bytes to the
*                       Hitachi HD44780 LCD controller. The user
*                       must check to see if the LCD controller is
*                       busy before calling this routine. The data
*                       is written to the character generator RAM or
*                       the display data RAM depending on what the
*                       previous SetxxRamAddr routine was called.
********************************************************************/
void putrsXLCD(const char *buffer)
{
        while(*buffer)                  // Write data to LCD up to null
        {
                //__delay_ms(1);      // Wait while LCD is busy
                WriteDataXLCD(*buffer); // Write character to LCD
                buffer++;               // Increment buffer
        }
        return;
}

/********************************************************************
*       Function Name:  putsXLCD
*       Return Value:   void
*       Parameters:     buffer: pointer to string
*       Description:    This routine writes a string of bytes to the
*                       Hitachi HD44780 LCD controller. The user
*                       must check to see if the LCD controller is
*                       busy before calling this routine. The data
*                       is written to the character generator RAM or
*                       the display data RAM depending on what the
*                       previous SetxxRamAddr routine was called.
********************************************************************/
void putsXLCD(char *buffer)
{
        while(*buffer)                  // Write data to LCD up to null
        {
                //__delay_ms(1);          // Wait while LCD is busy
                WriteDataXLCD(*buffer); // Write character to LCD
                buffer++;               // Increment buffer
        }
        return;
}

/********************************************************************
*       Function Name:  SetCGRamAddr                                *
*       Return Value:   void                                        *
*       Parameters:     CGaddr: character generator ram address     *
*       Description:    This routine sets the character generator   *
*                       address of the Hitachi HD44780 LCD          *
*                       controller. The user must check to see if   *
*                       the LCD controller is busy before calling   *
*                       this routine.                               *
********************************************************************/
void SetCGRamAddr(unsigned char CGaddr)
{
    //parte alta de la direccion
    CGaddr |= 0b01000000;
    
    //parte alta del comando
    dataOut = (CGaddr & 0xF0) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible alto 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    //parte baja del comando
    dataOut = (CGaddr << 4) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible bajo 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    __delay_us(100);
    

    return;
}

/********************************************************************
*       Function Name:  SetDDRamAddr                                *
*       Return Value:   void                                        *
*       Parameters:     CGaddr: display data address                *
*       Description:    This routine sets the display data address  *
*                       of the Hitachi HD44780 LCD controller. The  *
*                       user must check to see if the LCD controller*
*                       is busy before calling this routine.        *
********************************************************************/
void SetDDRamAddr(unsigned char DDaddr)
{
    //parte alta de la direccion
    DDaddr |= 0b10000000;
    
    //parte alta del comando
    dataOut = (DDaddr & 0xF0) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible alto 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    //parte baja del comando
    dataOut = (DDaddr << 4) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible bajo 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    __delay_us(100);
    

    return;
}

/********************************************************************
*       Function Name:  WriteCmdXLCD                                *
*       Return Value:   void                                        *
*       Parameters:     cmd: command to send to LCD                 *
*       Description:    This routine writes a command to the Hitachi*
*                       HD44780 LCD controller. The user must check *
*                       to see if the LCD controller is busy before *
*                       calling this routine.                       *
********************************************************************/
void WriteCmdXLCD(unsigned char cmd)
{        
    //parte alta del comando
    dataOut = (cmd & 0xF0) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible alto 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    //parte baja del comando
    dataOut = (cmd << 4) + stateLight;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible bajo 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b000000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    __delay_us(100);
        
    return;
}

/********************************************************************
*       Function Name:  WriteDataXLCD                               *
*       Return Value:   void                                        *
*       Parameters:     data: data byte to be written to LCD        *
*       Description:    This routine writes a data byte to the      *
*                       Hitachi HD44780 LCD controller. The user    *
*                       must check to see if the LCD controller is  *
*                       busy before calling this routine. The data  *
*                       is written to the character generator RAM or*
*                       the display data RAM depending on what the  *
*                       previous SetxxRamAddr routine was called.   *
********************************************************************/
void WriteDataXLCD(char data)
{
    //parte alta del dato
    dataOut = (data & 0xF0) + (stateLight | dataMask);
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible alto 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    //parte baja del comando
    dataOut = (data << 4) + (stateLight | dataMask);
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    
    dataOut += 0b00000100; // enviar nible bajo 
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    dataOut -= 0b00000100;
    I2C1_WriteNBytes(LCDaddress, &dataOut, 1);
    __delay_us(100);
    return;
}

/*Esta funcion borra el LCD y coloca el cursor en la direccion 0*/
void clearLCD(void)
{
    //primero aplicar Clear display
    dataOut = 0b00000001;
    WriteCmdXLCD(dataOut);
    __delay_ms(2);
      
    return;
    
}

void positionLCD(uint8_t line, uint8_t position)
{
    switch (line)
    {
        case 1:
            dataOut = position - 1;
            break;
        case 2:
            dataOut = (0x40 + position) - 1;
            break;
        case 3:
            dataOut = (0x14 + position) - 1;
            break;
        case 4:
            dataOut = (0x54 + position) - 1;
            break;
        default:
            #warning "line debe ser un numero entre 1 y 4"  // this works in XC8
            break;
    }
    
    SetDDRamAddr(dataOut);
}
