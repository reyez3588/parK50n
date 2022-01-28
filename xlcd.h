#ifndef __XLCD_H
#define __XLCD_H
#include <xc.h>
#include "../c99/stdint.h"
/* PIC18 XLCD peripheral routines.
 *
 *   Notes:
 *   
 *      - These libraries routines are written to support the
 *        Hitachi HD44780 LCD controller.
 */



/* Definir la direccion I2C a 7 bits */
#define LCDaddress 0b0100111 /*Direccion I2C del LCD*/

/*Variable en la que se guardara el dato a enviar por I2C*/
uint8_t dataOut;
uint8_t dataIn;

/*variable que controla la luz de fondo*/
uint8_t stateLight = 0b00001000; //luz de fondo encendida por default

/*Mascara para el nible bajo para el modulo I2C PCF8574*/
#define dataMask 0b00000001

/* Display ON/OFF Control defines */
#define DON         0b00001111  /* Display on      */
#define DOFF        0b00001011  /* Display off     */
#define CURSOR_ON   0b00001111  /* Cursor on       */
#define CURSOR_OFF  0b00001101  /* Cursor off      */
#define BLINK_ON    0b00001111  /* Cursor Blink    */
#define BLINK_OFF   0b00001110  /* Cursor No Blink */

/* Cursor or Display Shift defines */
#define SHIFT_CUR_LEFT    0b00000100  /* Cursor shifts to the left   */
#define SHIFT_CUR_RIGHT   0b00000101  /* Cursor shifts to the right  */
#define SHIFT_DISP_LEFT   0b00000110  /* Display shifts to the left  */
#define SHIFT_DISP_RIGHT  0b00000111  /* Display shifts to the right */

/* Function Set defines */
#define FOUR_BIT   0b00101100  /* 4-bit Interface               */
#define EIGHT_BIT  0b00111100  /* 8-bit Interface               */
#define LINE_5X7   0b00110000  /* 5x7 characters, single line   */
#define LINE_5X10  0b00110100  /* 5x10 characters               */
#define LINES_5X7  0b00111000  /* 5x7 characters, multiple line */

//#ifdef _OMNI_CODE_
//#define PARAM_SCLASS
//#else
//#define PARAM_SCLASS auto
//#endif
//
//#ifndef MEM_MODEL
//#ifdef _OMNI_CODE_
//#define MEM_MODEL
//#else
//#define MEM_MODEL far  /* Change this to near for small memory model */
//#endif
//#endif



/* OpenXLCD
 * Configures I/O pins for external LCD
 */
void OpenXLCD( uint8_t);


void initLCD(void); //inicializar LCD con parametros predefinidos


void toggleBacklight(void);
void backLightOFF(void);
void backLightON(void);
void clearLCD(void);
void positionLCD(uint8_t line, uint8_t position);


/* SetCGRamAddr
 * Sets the character generator address
 */
void SetCGRamAddr(uint8_t);

/* SetDDRamAddr
 * Sets the display data address
 */
void SetDDRamAddr(uint8_t);

/* BusyXLCD
 * Returns the busy status of the LCD
 */
unsigned char BusyXLCD(void);

/* WriteCmdXLCD
 * Writes a command to the LCD
 */
void WriteCmdXLCD(uint8_t);

/* WriteDataXLCD
 * Writes a data byte to the LCD
 */
void WriteDataXLCD(char);

/* putcXLCD
 * A putc is a write
 */
#define putcXLCD WriteDataXLCD

/* putsXLCD
 * Writes a string of characters to the LCD
 */
void putsXLCD(char *);

/* putrsXLCD
 * Writes a string of characters in ROM to the LCD
 */
void putrsXLCD(const char *);

#endif
