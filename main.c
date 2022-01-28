/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F45K50
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"

/*Librerias propias*/
#include "SparkFunLSM9DS1.h"
#include "../c99/math.h"
//#include "xlcd.h"
//#include "../c99/float.h"

volatile int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
volatile int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
volatile int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
volatile int16_t temperature; // Chip temperature

uint16_t IMUstate;
int contIMU=0;


typedef struct
{
	uint8_t initMessage;

	float accelX;
	float accelY;
	float accelZ;

	uint8_t finalMessage;
} accelData;


accelData myMessage;
accelData *ptrMessage = &myMessage;

/////////////////////////////////////
// Selecion de los datos de salida //
/////////////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 1 // 10 ms between prints


// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 3.49 // Declination (Degrees) Teziutlan Puebla

//Function definitions
void Accelfloat(void);
void printGyro(void);
void printAccel(void);
void printMag(void);
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);


/*
                         Main application
 */
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:

    // Enable high priority global interrupts
    INTERRUPT_GlobalInterruptHighEnable();

    // Enable low priority global interrupts.
    INTERRUPT_GlobalInterruptLowEnable();

    // Disable high priority global interrupts
    //INTERRUPT_GlobalInterruptHighDisable();

    // Disable low priority global interrupts.
    //INTERRUPT_GlobalInterruptLowDisable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    __delay_ms(1000);
    

    while(1)
    {
        __delay_ms(PRINT_SPEED);
        
        //USBDeviceTasks(); se realiza en la interrupcion
        if((USBGetDeviceState() < CONFIGURED_STATE) ||
           (USBIsDeviceSuspended() == true))
        {
            //Either the device is not configured or we are suspended
            //  so we don't want to do execute any application code
            continue;   //go back to the top of the while loop
        }
        else
        {
            //Keep trying to send data to the PC as required
            CDCTxService();
            contIMU = contIMU + 1; 
            if (contIMU == 1000)
            {
                IMUstate = LSM9DS1begin();
                contIMU = 0;
            }
            
            if (IMUstate == 0) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c.
            {
                if( USBUSARTIsTxTrfReady())
                {
                    putrsUSBUSART("Failed to communicate with LSM9DS1.\n");
                    //contIMU = 90; //para que la siguiente lectura la realice
                    
                }
            }

            if (IMUstate == 0x683D) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c.
            {
                // Update the sensor values whenever new data is available
                if ( accelAvailableALL() )
                {
                    // To read from the accelerometer, first call the
                    // readAccel() function. When it exits, it'll update the
                    // ax, ay, and az variables with the most current data.
                    readAccelALL();
                }
//                if ( gyroAvailableALL() )
//                {
//                    // To read from the gyroscope,  first call the
//                    // readGyro() function. When it exits, it'll update the
//                    // gx, gy, and gz variables with the most current data.
//                    readGyroALL();
//                }
//                
//                if ( magAvailable(ALL_AXIS) )
//                {
//                    // To read from the magnetometer, first call the
//                    // readMag() function. When it exits, it'll update the
//                    // mx, my, and mz variables with the most current data.
//                    readMagALL();
//                }
                
                if( USBUSARTIsTxTrfReady())
                {
//                    putrsUSBUSART("LSM9DS1 conectado!!!\n");
                LED_Toggle();

                //printGyro();  // Print "G: gx, gy, gz"
                //printAccel(); // Print "A: ax, ay, az"
                //printMag();   // Print "M: mx, my, mz"
                //printAttitude(ax, ay, az, -my, -mx, mz);
                Accelfloat();

                }

            }    

        }                

    }
}
/**
 End of File
*/

void printGyro()
{
   char strGyro[64];
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
//  putrsUSBUSART("G: ");
  
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
   
  sprintf(strGyro, "G: %.2f, %.2f, %.2f deg/s \n", calcGyro(gx), calcGyro(gy), calcGyro(gz));
  putsUSBUSART(strGyro);
  
#elif defined PRINT_RAW
  sprintf(strGyro, "G: %i, %i, %i \n", gx, gy, gz);
  putsUSBUSART(strGyro);

#endif
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  char strAccel[64];
  
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.

  sprintf(strAccel, "A: %.2f, %.2f, %.2f g \n", calcAccel(ax), calcAccel(ay), calcAccel(az));
  putsUSBUSART(strAccel);
  
#elif defined PRINT_RAW
  sprintf(strAccel, "A: %i, %i, %i \n", ax, ay, az);
  putsUSBUSART(strAccel);
  
#endif

}

void printMag()
{
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  char strMag[64];
  
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.

  sprintf(strMag, "M: %.2f, %.2f, %.2f gauss \n", calcMag(mx), calcMag(my), calcMag(mz));
  putsUSBUSART(strMag);
  
#elif defined PRINT_RAW
  
  sprintf(strMag, "M: %i, %i, %i \n", mx, my, mz);
  putsUSBUSART(strMag);
  
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    char strAttitude[64];
    
    float PI = 3.1416;
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;
    if (my == 0)
    heading = (mx < 0) ? PI : 0;
    else
    heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;

    sprintf(strAttitude, "Pitch %.2f, Roll: %.2f, Heading: %.2f \n", pitch, roll, heading);
    putsUSBUSART(strAttitude);

}

/*Esta funcion se envia un mensaje con la estructura adecuada para DataVisualizer,*/
void Accelfloat(void)
{
    
    myMessage.initMessage = 0x5F;
    //myMessage.accelX = calcAccel(ax);
    //myMessage.accelY = calcAccel(ay);
    //myMessage.accelZ = calcAccel(az);
    ptrMessage -> accelX = calcAccel(ax);
    ptrMessage -> accelY = calcAccel(ay);
    ptrMessage -> accelZ = calcAccel(az);
    //myMessage.finalMessage = 0xA0;
    
    //uint8_t *pData = &myMessage.initMessage;
    
    putUSBUSART((uint8_t *)ptrMessage,14); //,
}