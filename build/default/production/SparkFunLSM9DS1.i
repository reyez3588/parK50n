# 1 "SparkFunLSM9DS1.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "SparkFunLSM9DS1.c" 2
# 21 "SparkFunLSM9DS1.c"
# 1 "./SparkFunLSM9DS1.h" 1
# 21 "./SparkFunLSM9DS1.h"
# 1 "./LSM9DS1_Registers.h" 1
# 22 "./SparkFunLSM9DS1.h" 2
# 1 "./LSM9DS1_Types.h" 1
# 21 "./LSM9DS1_Types.h"
# 1 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 1 3



# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/musl_xc8.h" 1 3
# 5 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 2 3
# 22 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3
# 127 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long uintptr_t;
# 142 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long intptr_t;
# 158 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;





typedef long long int64_t;
# 188 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 229 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 23 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 139 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 140 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdint.h" 2 3
# 22 "./LSM9DS1_Types.h" 2
# 1 "/opt/microchip/mplabx/v5.50/packs/Microchip/PIC18F-K_DFP/1.4.87/xc8/pic/include/proc/../c99/stdbool.h" 1 3
# 23 "./LSM9DS1_Types.h" 2





enum interface_mode
{
 IMU_MODE_SPI,
 IMU_MODE_I2C,
};


enum accel_scale
{
 A_SCALE_2G,
 A_SCALE_16G,
 A_SCALE_4G,
 A_SCALE_8G
};


enum gyro_scale
{
 G_SCALE_245DPS,
 G_SCALE_500DPS,
 G_SCALE_2000DPS,
};


enum mag_scale
{
 M_SCALE_4GS,
 M_SCALE_8GS,
 M_SCALE_12GS,
 M_SCALE_16GS,
};


enum gyro_odr
{

 G_ODR_PD,
 G_ODR_149,
 G_ODR_595,
 G_ODR_119,
 G_ODR_238,
 G_ODR_476,
 G_ODR_952
};

enum accel_odr
{
 XL_POWER_DOWN,
 XL_ODR_10,
 XL_ODR_50,
 XL_ODR_119,
 XL_ODR_238,
 XL_ODR_476,
 XL_ODR_952
};


enum accel_abw
{
 A_ABW_408,
 A_ABW_211,
 A_ABW_105,
 A_ABW_50,
};


enum mag_odr
{
 M_ODR_0625,
 M_ODR_125,
 M_ODR_250,
 M_ODR_5,
 M_ODR_10,
 M_ODR_20,
 M_ODR_40,
 M_ODR_80
};

enum interrupt_select
{
 XG_INT1 = 0x0C,
 XG_INT2 = 0x0D
};

enum interrupt_generators
{
 INT_DRDY_XL = (1 << 0),
 INT_DRDY_G = (1 << 1),
 INT1_BOOT = (1 << 2),
 INT2_DRDY_TEMP = (1 << 2),
 INT_FTH = (1 << 3),
 INT_OVR = (1 << 4),
 INT_FSS5 = (1 << 5),
 INT_IG_XL = (1 << 6),
 INT1_IG_G = (1 << 7),
 INT2_INACT = (1 << 7),
};

enum accel_interrupt_generator
{
 XLIE_XL = (1 << 0),
 XHIE_XL = (1 << 1),
 YLIE_XL = (1 << 2),
 YHIE_XL = (1 << 3),
 ZLIE_XL = (1 << 4),
 ZHIE_XL = (1 << 5),
 GEN_6D = (1 << 6)
};

enum gyro_interrupt_generator
{
 XLIE_G = (1 << 0),
 XHIE_G = (1 << 1),
 YLIE_G = (1 << 2),
 YHIE_G = (1 << 3),
 ZLIE_G = (1 << 4),
 ZHIE_G = (1 << 5)
};

enum mag_interrupt_generator
{
 ZIEN = (1 << 5),
 YIEN = (1 << 6),
 XIEN = (1 << 7)
};

enum h_lactive
{
 INT_ACTIVE_HIGH,
 INT_ACTIVE_LOW
};

enum pp_od
{
 INT_PUSH_PULL,
 INT_OPEN_DRAIN
};

enum fifoMode_type
{
 FIFO_OFF = 0,
 FIFO_THS = 1,
 FIFO_CONT_TRIGGER = 3,
 FIFO_OFF_TRIGGER = 4,
 FIFO_CONT = 6
};

struct gyroSettings
{

 uint8_t enabled;
 uint16_t scale;
 uint8_t sampleRate;

 uint8_t bandwidth;
 uint8_t lowPowerEnable;
 uint8_t HPFEnable;
 uint8_t HPFCutoff;
 uint8_t flipX;
 uint8_t flipY;
 uint8_t flipZ;
 uint8_t orientation;
 uint8_t enableX;
 uint8_t enableY;
 uint8_t enableZ;
 uint8_t latchInterrupt;
};

struct deviceSettings
{
 uint8_t commInterface;
 uint8_t agAddress;
 uint8_t mAddress;
    _Bool i2c;
};

struct accelSettings
{

 uint8_t enabled;
 uint8_t scale;
 uint8_t sampleRate;

 uint8_t enableX;
 uint8_t enableY;
 uint8_t enableZ;
 int8_t bandwidth;
 uint8_t highResEnable;
 uint8_t highResBandwidth;
};

struct magSettings
{

 uint8_t enabled;
 uint8_t scale;
 uint8_t sampleRate;

 uint8_t tempCompensationEnable;
 uint8_t XYPerformance;
 uint8_t ZPerformance;
 uint8_t lowPowerEnable;
 uint8_t operatingMode;
};

struct temperatureSettings
{

 uint8_t enabled;
};

typedef struct
{
 struct deviceSettings device;

 struct gyroSettings gyro;
 struct accelSettings accel;
 struct magSettings mag;

 struct temperatureSettings temp;
}IMUconfig;
# 23 "./SparkFunLSM9DS1.h" 2







enum lsm9ds1_axis {
 X_AXIS,
 Y_AXIS,
 Z_AXIS,
 ALL_AXIS
};
# 47 "./SparkFunLSM9DS1.h"
extern volatile int16_t gx, gy, gz;
extern volatile int16_t ax, ay, az;
extern volatile int16_t mx, my, mz;
extern volatile int16_t temperature;
float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];
# 67 "./SparkFunLSM9DS1.h"
 uint16_t LSM9DS1begin(void);



    void calibrate(_Bool autoCalc);
 void calibrateMag(_Bool loadIn);
 void magOffset(uint8_t axis, int16_t offset);





 uint8_t accelAvailableALL(void);





 uint8_t gyroAvailableALL(void);





 uint8_t tempAvailable(void);
# 101 "./SparkFunLSM9DS1.h"
 uint8_t magAvailable(enum lsm9ds1_axis axis);





 void readGyroALL(void);







 int16_t readGyro(enum lsm9ds1_axis axis);





 void readAccelALL(void);







 int16_t readAccel(enum lsm9ds1_axis axis);





 void readMagALL(void);







 int16_t readMag(enum lsm9ds1_axis axis);





 void readTemp(void);






 float calcGyro(int16_t gyro);






 float calcAccel(int16_t accel);






 float calcMag(int16_t mag);







 void setGyroScale(uint16_t gScl);







 void setAccelScale(uint8_t aScl);







 void setMagScale(uint8_t mScl);




 void setGyroODR(uint8_t gRate);




 void setAccelODR(uint8_t aRate);




 void setMagODR(uint8_t mRate);
# 218 "./SparkFunLSM9DS1.h"
 void configInactivity(uint8_t duration, uint8_t threshold, _Bool sleepOn);
# 227 "./SparkFunLSM9DS1.h"
 void configAccelInt(uint8_t generator, _Bool andInterrupts);
# 238 "./SparkFunLSM9DS1.h"
 void configAccelThs(uint8_t threshold, enum lsm9ds1_axis axis, uint8_t duration, _Bool wait);
# 248 "./SparkFunLSM9DS1.h"
 void configGyroInt(uint8_t generator, _Bool aoi, _Bool latch);
# 259 "./SparkFunLSM9DS1.h"
 void configGyroThs(int16_t threshold, enum lsm9ds1_axis axis, uint8_t duration, _Bool wait);
# 274 "./SparkFunLSM9DS1.h"
    void configInt(enum interrupt_select interupt, uint8_t generator,
       enum h_lactive activeLow, enum pp_od pushPull);
# 284 "./SparkFunLSM9DS1.h"
 void configMagInt(uint8_t generator, enum h_lactive activeLow, _Bool latch);





 void configMagThs(uint16_t threshold);


 uint8_t getGyroIntSrc(void);


 uint8_t getAccelIntSrc(void);


 uint8_t getMagIntSrc(void);


 uint8_t getInactivity(void);




 void sleepGyro(_Bool enable);




 void enableFIFO(_Bool enable);







 void setFIFO(enum fifoMode_type fifoMode, uint8_t fifoThs);


 uint8_t getFIFOSamples(void);





 uint8_t _mAddress, _xgAddress;




 float gRes, aRes, mRes;



 _Bool _autoCalc;



 void LSM9DS1_defaultConfig(void);
# 356 "./SparkFunLSM9DS1.h"
 void initGyro(void);
# 366 "./SparkFunLSM9DS1.h"
 void initAccel(void);
# 377 "./SparkFunLSM9DS1.h"
 void initMag(void);






 uint8_t mReadByte(uint8_t subAddress);
# 395 "./SparkFunLSM9DS1.h"
 uint8_t mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);





 void mWriteByte(uint8_t subAddress, uint8_t data);






 uint8_t xgReadByte(uint8_t subAddress);
# 419 "./SparkFunLSM9DS1.h"
 uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);





 void xgWriteByte(uint8_t subAddress, uint8_t data);




 void calcgRes(void);




 void calcmRes(void);




 void calcaRes(void);




 void constrainScales(void);
# 457 "./SparkFunLSM9DS1.h"
 void LSM9DS1_I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);







 uint8_t LSM9DS1_I2CreadByte(uint8_t address, uint8_t subAddress);
# 475 "./SparkFunLSM9DS1.h"
 uint8_t LSM9DS1_I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
# 22 "SparkFunLSM9DS1.c" 2




# 1 "./mcc_generated_files/examples/i2c1_master_example.h" 1
# 50 "./mcc_generated_files/examples/i2c1_master_example.h"
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 1 3
# 50 "./mcc_generated_files/examples/i2c1_master_example.h" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 1 3
# 10 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/features.h" 1 3
# 11 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 2 3
# 24 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 3
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 1 3





typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 122 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef unsigned size_t;
# 137 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long ssize_t;
# 246 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef long long off_t;
# 399 "/opt/microchip/xc8/v2.32/pic/include/c99/bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 25 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 2 3
# 52 "/opt/microchip/xc8/v2.32/pic/include/c99/stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);
int ungetc(int, FILE *);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

#pragma printf_check(printf) const
#pragma printf_check(vprintf) const
#pragma printf_check(sprintf) const
#pragma printf_check(snprintf) const
#pragma printf_check(vsprintf) const
#pragma printf_check(vsnprintf) const

int printf(const char *restrict, ...);
int fprintf(FILE *restrict, const char *restrict, ...);
int sprintf(char *restrict, const char *restrict, ...);
int snprintf(char *restrict, size_t, const char *restrict, ...);

int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

int scanf(const char *restrict, ...);
int fscanf(FILE *restrict, const char *restrict, ...);
int sscanf(const char *restrict, const char *restrict, ...);
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 51 "./mcc_generated_files/examples/i2c1_master_example.h" 2

# 1 "./mcc_generated_files/examples/../i2c1_master.h" 1
# 55 "./mcc_generated_files/examples/../i2c1_master.h"
# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdint.h" 1 3
# 55 "./mcc_generated_files/examples/../i2c1_master.h" 2

# 1 "/opt/microchip/xc8/v2.32/pic/include/c99/stdbool.h" 1 3
# 56 "./mcc_generated_files/examples/../i2c1_master.h" 2


typedef enum {
    I2C1_NOERR,
    I2C1_BUSY,
    I2C1_FAIL


} i2c1_error_t;

typedef enum
{
    I2C1_STOP=1,
    I2C1_RESTART_READ,
    I2C1_RESTART_WRITE,
    I2C1_CONTINUE,
    I2C1_RESET_LINK
} i2c1_operations_t;

typedef uint8_t i2c1_address_t;
typedef i2c1_operations_t (*i2c1_callback_t)(void *funPtr);


i2c1_operations_t I2C1_CallbackReturnStop(void *funPtr);
i2c1_operations_t I2C1_CallbackReturnReset(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartWrite(void *funPtr);
i2c1_operations_t I2C1_CallbackRestartRead(void *funPtr);






void I2C1_Initialize(void);
# 101 "./mcc_generated_files/examples/../i2c1_master.h"
i2c1_error_t I2C1_Open(i2c1_address_t address);
# 111 "./mcc_generated_files/examples/../i2c1_master.h"
i2c1_error_t I2C1_Close(void);
# 123 "./mcc_generated_files/examples/../i2c1_master.h"
i2c1_error_t I2C1_MasterOperation(_Bool read);




i2c1_error_t I2C1_MasterWrite(void);




i2c1_error_t I2C1_MasterRead(void);
# 142 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetTimeout(uint8_t timeOut);
# 152 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetBuffer(void *buffer, size_t bufferSize);
# 164 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetDataCompleteCallback(i2c1_callback_t cb, void *ptr);
# 174 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetWriteCollisionCallback(i2c1_callback_t cb, void *ptr);
# 184 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetAddressNackCallback(i2c1_callback_t cb, void *ptr);
# 194 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetDataNackCallback(i2c1_callback_t cb, void *ptr);
# 204 "./mcc_generated_files/examples/../i2c1_master.h"
void I2C1_SetTimeoutCallback(i2c1_callback_t cb, void *ptr);
# 52 "./mcc_generated_files/examples/i2c1_master_example.h" 2


uint8_t I2C1_Read1ByteRegister(i2c1_address_t address, uint8_t reg);
uint16_t I2C1_Read2ByteRegister(i2c1_address_t address, uint8_t reg);
void I2C1_Write1ByteRegister(i2c1_address_t address, uint8_t reg, uint8_t data);
void I2C1_Write2ByteRegister(i2c1_address_t address, uint8_t reg, uint16_t data);
void I2C1_WriteNBytes(i2c1_address_t address, uint8_t *data, size_t len);
void I2C1_ReadNBytes(i2c1_address_t address, uint8_t *data, size_t len);
void I2C1_ReadDataBlock(i2c1_address_t address, uint8_t reg, uint8_t *data, size_t len);
# 27 "SparkFunLSM9DS1.c" 2
# 46 "SparkFunLSM9DS1.c"
volatile IMUconfig IMUSettings;

void LSM9DS1_defaultConfig()
{

    IMUSettings.gyro.enabled = 1;
    IMUSettings.gyro.enableX = 1;
    IMUSettings.gyro.enableY = 1;
    IMUSettings.gyro.enableZ = 1;


    IMUSettings.gyro.scale = 245;





    IMUSettings.gyro.sampleRate = 6;




    IMUSettings.gyro.bandwidth = 0;

    IMUSettings.gyro.lowPowerEnable = 0;
    IMUSettings.gyro.HPFEnable = 0;




    IMUSettings.gyro.HPFCutoff = 0;

    IMUSettings.gyro.flipX = 0;
    IMUSettings.gyro.flipY = 0;
    IMUSettings.gyro.flipZ = 0;
    IMUSettings.gyro.orientation = 0;
    IMUSettings.gyro.latchInterrupt = 1;


    IMUSettings.accel.enabled = 1;
    IMUSettings.accel.enableX = 1;
    IMUSettings.accel.enableY = 1;
    IMUSettings.accel.enableZ = 1;


    IMUSettings.accel.scale = 2;





    IMUSettings.accel.sampleRate = 6;





    IMUSettings.accel.bandwidth = -1;
    IMUSettings.accel.highResEnable = 0;





    IMUSettings.accel.highResBandwidth = 0;

    IMUSettings.mag.enabled = 1;


    IMUSettings.mag.scale = 4;






    IMUSettings.mag.sampleRate = 7;
    IMUSettings.mag.tempCompensationEnable = 0;




    IMUSettings.mag.XYPerformance = 3;

    IMUSettings.mag.ZPerformance = 3;
    IMUSettings.mag.lowPowerEnable = 0;





    IMUSettings.mag.operatingMode = 0;

    IMUSettings.temp.enabled = 1;
 for (int i=0; i<3; i++)
 {
  gBias[i] = 0;
  aBias[i] = 0;
  mBias[i] = 0;
  gBiasRaw[i] = 0;
  aBiasRaw[i] = 0;
  mBiasRaw[i] = 0;
 }
 _autoCalc = 0;
}


uint16_t LSM9DS1begin()
{


    IMUSettings.device.commInterface = IMU_MODE_I2C;
    IMUSettings.device.agAddress = 0x6B;
    IMUSettings.device.mAddress = 0x1E;
    IMUSettings.device.i2c = 1;


 _xgAddress = IMUSettings.device.agAddress;
 _mAddress = IMUSettings.device.mAddress;

    LSM9DS1_defaultConfig();

 constrainScales();


 calcgRes();
 calcmRes();
 calcaRes();







 uint8_t mTest = mReadByte(0x0F);
 uint8_t xgTest = xgReadByte(0x0F);
 uint16_t whoAmICombined = (xgTest << 8) | mTest;

 if (whoAmICombined != ((0x68 << 8) | 0x3D))
  return 0;


 initGyro();


 initAccel();


 initMag();


 return whoAmICombined;
}
# 246 "SparkFunLSM9DS1.c"
void initGyro()
{
 uint8_t tempRegValue = 0;
# 258 "SparkFunLSM9DS1.c"
 if (IMUSettings.gyro.enabled)
 {
  tempRegValue = (IMUSettings.gyro.sampleRate & 0x07) << 5;
 }
 switch (IMUSettings.gyro.scale)
 {
  case 500:
   tempRegValue |= (0x1 << 3);
   break;
  case 2000:
   tempRegValue |= (0x3 << 3);
   break;

 }
 tempRegValue |= (IMUSettings.gyro.bandwidth & 0x3);
 xgWriteByte(0x10, tempRegValue);





 xgWriteByte(0x11, 0x00);






 tempRegValue = IMUSettings.gyro.lowPowerEnable ? (1<<7) : 0;
 if (IMUSettings.gyro.HPFEnable)
 {
  tempRegValue |= (1<<6) | (IMUSettings.gyro.HPFCutoff & 0x0F);
 }
 xgWriteByte(0x12, tempRegValue);
# 300 "SparkFunLSM9DS1.c"
 tempRegValue = 0;
 if (IMUSettings.gyro.enableZ) tempRegValue |= (1<<5);
 if (IMUSettings.gyro.enableY) tempRegValue |= (1<<4);
 if (IMUSettings.gyro.enableX) tempRegValue |= (1<<3);
 if (IMUSettings.gyro.latchInterrupt) tempRegValue |= (1<<1);
 xgWriteByte(0x1E, tempRegValue);





 tempRegValue = 0;
 if (IMUSettings.gyro.flipX) tempRegValue |= (1<<5);
 if (IMUSettings.gyro.flipY) tempRegValue |= (1<<4);
 if (IMUSettings.gyro.flipZ) tempRegValue |= (1<<3);
 xgWriteByte(0x13, tempRegValue);
}

void initAccel()
{
 uint8_t tempRegValue = 0;
# 329 "SparkFunLSM9DS1.c"
 if (IMUSettings.accel.enableZ) tempRegValue |= (1<<5);
 if (IMUSettings.accel.enableY) tempRegValue |= (1<<4);
 if (IMUSettings.accel.enableX) tempRegValue |= (1<<3);

 xgWriteByte(0x1F, tempRegValue);







 tempRegValue = 0;

 if (IMUSettings.accel.enabled)
 {
  tempRegValue |= (IMUSettings.accel.sampleRate & 0x07) << 5;
 }
 switch (IMUSettings.accel.scale)
 {
  case 4:
   tempRegValue |= (0x2 << 3);
   break;
  case 8:
   tempRegValue |= (0x3 << 3);
   break;
  case 16:
   tempRegValue |= (0x1 << 3);
   break;

 }
 if (IMUSettings.accel.bandwidth >= 0)
 {
  tempRegValue |= (1<<2);
  tempRegValue |= (IMUSettings.accel.bandwidth & 0x03);
 }
 xgWriteByte(0x20, tempRegValue);







 tempRegValue = 0;
 if (IMUSettings.accel.highResEnable)
 {
  tempRegValue |= (1<<7);
  tempRegValue |= (IMUSettings.accel.highResBandwidth & 0x3) << 5;
 }
 xgWriteByte(0x21, tempRegValue);
}
# 389 "SparkFunLSM9DS1.c"
void calibrate(_Bool autoCalc)
{
 uint8_t samples = 0;
 int ii;
 int32_t aBiasRawTemp[3] = {0, 0, 0};
 int32_t gBiasRawTemp[3] = {0, 0, 0};


 enableFIFO(1);
 setFIFO(FIFO_THS, 0x1F);
 while (samples < 0x1F)
 {
  samples = (xgReadByte(0x2F) & 0x3F);
 }
 for(ii = 0; ii < samples ; ii++)
 {
  readGyroALL();
  gBiasRawTemp[0] += gx;
  gBiasRawTemp[1] += gy;
  gBiasRawTemp[2] += gz;
  readAccelALL();
  aBiasRawTemp[0] += ax;
  aBiasRawTemp[1] += ay;
  aBiasRawTemp[2] += az - (int16_t)(1./aRes);
 }
 for (ii = 0; ii < 3; ii++)
 {
  gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
  gBias[ii] = calcGyro(gBiasRaw[ii]);
  aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
  aBias[ii] = calcAccel(aBiasRaw[ii]);
 }

 enableFIFO(0);
 setFIFO(FIFO_OFF, 0x00);

 if (autoCalc) _autoCalc = 1;
}

void LSM9DS1calibrateMag(_Bool loadIn)
{
 int i, j;
 int16_t magMin[3] = {0, 0, 0};
 int16_t magMax[3] = {0, 0, 0};

 for (i=0; i<128; i++)
 {
  while (!magAvailable(ALL_AXIS))
   ;
  readMagALL();
  int16_t magTemp[3] = {0, 0, 0};
  magTemp[0] = mx;
  magTemp[1] = my;
  magTemp[2] = mz;
  for (j = 0; j < 3; j++)
  {
   if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
   if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
  }
 }
 for (j = 0; j < 3; j++)
 {
  mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
  mBias[j] = calcMag(mBiasRaw[j]);
  if (loadIn)
   magOffset(j, mBiasRaw[j]);
 }

}
void magOffset(uint8_t axis, int16_t offset)
{
 if (axis > 2)
  return;
 uint8_t msb, lsb;
 msb = (offset & 0xFF00) >> 8;
 lsb = offset & 0x00FF;
 mWriteByte(0x05 + (2 * axis), lsb);
 mWriteByte(0x06 + (2 * axis), msb);
}

void initMag()
{
 uint8_t tempRegValue = 0;
# 481 "SparkFunLSM9DS1.c"
 if (IMUSettings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
 tempRegValue |= (IMUSettings.mag.XYPerformance & 0x3) << 5;
 tempRegValue |= (IMUSettings.mag.sampleRate & 0x7) << 2;
 mWriteByte(0x20, tempRegValue);






 tempRegValue = 0;
 switch (IMUSettings.mag.scale)
 {
 case 8:
  tempRegValue |= (0x1 << 5);
  break;
 case 12:
  tempRegValue |= (0x2 << 5);
  break;
 case 16:
  tempRegValue |= (0x3 << 5);
  break;

 }
 mWriteByte(0x21, tempRegValue);
# 515 "SparkFunLSM9DS1.c"
 tempRegValue = 0;
 if (IMUSettings.mag.lowPowerEnable) tempRegValue |= (1<<5);
 tempRegValue |= (IMUSettings.mag.operatingMode & 0x3);
 mWriteByte(0x22, tempRegValue);







 tempRegValue = 0;
 tempRegValue = (IMUSettings.mag.ZPerformance & 0x3) << 2;
 mWriteByte(0x23, tempRegValue);





 tempRegValue = 0;
 mWriteByte(0x24, tempRegValue);
}

uint8_t accelAvailableALL()
{
 uint8_t status = xgReadByte(0x27);

 return (status & (1<<0));
}

uint8_t gyroAvailableALL()
{
 uint8_t status = xgReadByte(0x27);

 return ((status & (1<<1)) >> 1);
}

uint8_t tempAvailable()
{
 uint8_t status = xgReadByte(0x27);

 return ((status & (1<<2)) >> 2);
}

uint8_t magAvailable(enum lsm9ds1_axis axis)
{
 uint8_t status;
 status = mReadByte(0x27);

 return ((status & (1<<axis)) >> axis);
}

void readAccelALL()
{
 uint8_t temp[6];
 if ( xgReadBytes(0x28, temp, 6) == 6 )
 {
  ax = (temp[1] << 8) | temp[0];
  ay = (temp[3] << 8) | temp[2];
  az = (temp[5] << 8) | temp[4];
  if (_autoCalc)
  {
   ax -= aBiasRaw[X_AXIS];
   ay -= aBiasRaw[Y_AXIS];
   az -= aBiasRaw[Z_AXIS];
  }
 }
}

int16_t readAccel(enum lsm9ds1_axis axis)
{
 uint8_t temp[2];
 int16_t value;
 if ( xgReadBytes(0x28 + (2 * axis), temp, 2) == 2)
 {
  value = (temp[1] << 8) | temp[0];

  if (_autoCalc)
   value -= aBiasRaw[axis];

  return value;
 }
 return 0;
}

void readMagALL()
{
 uint8_t temp[6];
 if ( mReadBytes(0x28, temp, 6) == 6)
 {
  mx = (temp[1] << 8) | temp[0];
  my = (temp[3] << 8) | temp[2];
  mz = (temp[5] << 8) | temp[4];
 }
}

int16_t readMag(enum lsm9ds1_axis axis)
{
 uint8_t temp[2];
 if ( mReadBytes(0x28 + (2 * axis), temp, 2) == 2)
 {
  return (temp[1] << 8) | temp[0];
 }
 return 0;
}

void readTemp()
{
 uint8_t temp[2];
 if ( xgReadBytes(0x15, temp, 2) == 2 )
 {
  int16_t offset = 25;
  temperature = offset + ((((int16_t)temp[1] << 8) | temp[0]) >> 8) ;
 }
}

void readGyroALL()
{
 uint8_t temp[6];
 if ( xgReadBytes(0x18, temp, 6) == 6)
 {
  gx = (temp[1] << 8) | temp[0];
  gy = (temp[3] << 8) | temp[2];
  gz = (temp[5] << 8) | temp[4];
  if (_autoCalc)
  {
   gx -= gBiasRaw[X_AXIS];
   gy -= gBiasRaw[Y_AXIS];
   gz -= gBiasRaw[Z_AXIS];
  }
 }
}

int16_t readGyro(enum lsm9ds1_axis axis)
{
 uint8_t temp[2];
 int16_t value;

 if ( xgReadBytes(0x18 + (2 * axis), temp, 2) == 2)
 {
  value = (temp[1] << 8) | temp[0];

  if (_autoCalc)
   value -= gBiasRaw[axis];

  return value;
 }
 return 0;
}

float calcGyro(int16_t gyro)
{

 return gRes * gyro;
}

float calcAccel(int16_t accel)
{

 return aRes * accel;
}

float calcMag(int16_t mag)
{

 return mRes * mag;
}

void setGyroScale(uint16_t gScl)
{

 uint8_t ctrl1RegValue = xgReadByte(0x10);

 ctrl1RegValue &= 0xE7;
 switch (gScl)
 {
  case 500:
   ctrl1RegValue |= (0x1 << 3);
   IMUSettings.gyro.scale = 500;
   break;
  case 2000:
   ctrl1RegValue |= (0x3 << 3);
   IMUSettings.gyro.scale = 2000;
   break;
  default:
   IMUSettings.gyro.scale = 245;
   break;
 }
 xgWriteByte(0x10, ctrl1RegValue);

 calcgRes();
}

void setAccelScale(uint8_t aScl)
{

 uint8_t tempRegValue = xgReadByte(0x20);

 tempRegValue &= 0xE7;

 switch (aScl)
 {
  case 4:
   tempRegValue |= (0x2 << 3);
   IMUSettings.accel.scale = 4;
   break;
  case 8:
   tempRegValue |= (0x3 << 3);
   IMUSettings.accel.scale = 8;
   break;
  case 16:
   tempRegValue |= (0x1 << 3);
   IMUSettings.accel.scale = 16;
   break;
  default:
   IMUSettings.accel.scale = 2;
   break;
 }
 xgWriteByte(0x20, tempRegValue);


 calcaRes();
}

void setMagScale(uint8_t mScl)
{

 uint8_t temp = mReadByte(0x21);

 temp &= 0xFF^(0x3 << 5);

 switch (mScl)
 {
 case 8:
  temp |= (0x1 << 5);
  IMUSettings.mag.scale = 8;
  break;
 case 12:
  temp |= (0x2 << 5);
  IMUSettings.mag.scale = 12;
  break;
 case 16:
  temp |= (0x3 << 5);
  IMUSettings.mag.scale = 16;
  break;
 default:
  IMUSettings.mag.scale = 4;
  break;
 }


 mWriteByte(0x21, temp);





 calcmRes();
}

void setGyroODR(uint8_t gRate)
{

 if ((gRate & 0x07) != 0)
 {

  uint8_t temp = xgReadByte(0x10);

  temp &= 0xFF^(0x7 << 5);
  temp |= (gRate & 0x07) << 5;

  IMUSettings.gyro.sampleRate = gRate & 0x07;

  xgWriteByte(0x10, temp);
 }
}

void setAccelODR(uint8_t aRate)
{

 if ((aRate & 0x07) != 0)
 {

  uint8_t temp = xgReadByte(0x20);

  temp &= 0x1F;

  temp |= ((aRate & 0x07) << 5);
  IMUSettings.accel.sampleRate = aRate & 0x07;

  xgWriteByte(0x20, temp);
 }
}

void setMagODR(uint8_t mRate)
{

 uint8_t temp = mReadByte(0x20);

 temp &= 0xFF^(0x7 << 2);

 temp |= ((mRate & 0x07) << 2);
 IMUSettings.mag.sampleRate = mRate & 0x07;

 mWriteByte(0x20, temp);
}

void calcgRes()
{
 switch (IMUSettings.gyro.scale)
 {
 case 245:
  gRes = 0.00875;
  break;
 case 500:
  gRes = 0.0175;
  break;
 case 2000:
  gRes = 0.07;
  break;
 default:
  break;
 }
}

void calcaRes()
{
 switch (IMUSettings.accel.scale)
 {
 case 2:
  aRes = 0.000061;
  break;
 case 4:
  aRes = 0.000122;
  break;
 case 8:
  aRes = 0.000244;
  break;
 case 16:
  aRes = 0.000732;
  break;
 default:
  break;
 }
}

void calcmRes()
{
 switch (IMUSettings.mag.scale)
 {
 case 4:
  mRes = 0.00014;
  break;
 case 8:
  mRes = 0.00029;
  break;
 case 12:
  mRes = 0.00043;
  break;
 case 16:
  mRes = 0.00058;
  break;
 }
}

void configInt(enum interrupt_select interrupt, uint8_t generator,
                      enum h_lactive activeLow, enum pp_od pushPull)
{



 xgWriteByte(interrupt, generator);


 uint8_t temp;
 temp = xgReadByte(0x22);

 if (activeLow) temp |= (1<<5);
 else temp &= ~(1<<5);

 if (pushPull) temp &= ~(1<<4);
 else temp |= (1<<4);

 xgWriteByte(0x22, temp);
}

void configInactivity(uint8_t duration, uint8_t threshold, _Bool sleepOn)
{
 uint8_t temp = 0;

 temp = threshold & 0x7F;
 if (sleepOn) temp |= (1<<7);
 xgWriteByte(0x04, temp);

 xgWriteByte(0x05, duration);
}

uint8_t getInactivity()
{
 uint8_t temp = xgReadByte(0x17);
 temp &= (0x10);
 return temp;
}

void configAccelInt(uint8_t generator, _Bool andInterrupts)
{


 uint8_t temp = generator;
 if (andInterrupts) temp |= 0x80;
 xgWriteByte(0x06, temp);
}

void configAccelThs(uint8_t threshold, enum lsm9ds1_axis axis, uint8_t duration, _Bool wait)
{


 xgWriteByte(0x07 + axis, threshold);


 uint8_t temp;
 temp = (duration & 0x7F);
 if (wait) temp |= 0x80;
 xgWriteByte(0x0A, temp);
}

uint8_t getAccelIntSrc()
{
 uint8_t intSrc = xgReadByte(0x26);


 if (intSrc & (1<<6))
 {
  return (intSrc & 0x3F);
 }

 return 0;
}

void configGyroInt(uint8_t generator, _Bool aoi, _Bool latch)
{


 uint8_t temp = generator;
 if (aoi) temp |= 0x80;
 if (latch) temp |= 0x40;
 xgWriteByte(0x30, temp);
}

void configGyroThs(int16_t threshold, enum lsm9ds1_axis axis, uint8_t duration, _Bool wait)
{
 uint8_t buffer[2];
 buffer[0] = (threshold & 0x7F00) >> 8;
 buffer[1] = (threshold & 0x00FF);


 xgWriteByte(0x31 + (axis * 2), buffer[0]);
 xgWriteByte(0x31 + 1 + (axis * 2), buffer[1]);


 uint8_t temp;
 temp = (duration & 0x7F);
 if (wait) temp |= 0x80;
 xgWriteByte(0x37, temp);
}

uint8_t getGyroIntSrc()
{
 uint8_t intSrc = xgReadByte(0x14);


 if (intSrc & (1<<6))
 {
  return (intSrc & 0x3F);
 }

 return 0;
}

void configMagInt(uint8_t generator, enum h_lactive activeLow, _Bool latch)
{

 uint8_t config = (generator & 0xE0);

 if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);

 if (!latch) config |= (1<<1);

 if (generator != 0) config |= (1<<0);

 mWriteByte(0x30, config);
}

void configMagThs(uint16_t threshold)
{

    uint8_t thresholdHigh = (threshold & 0x7F00) >> 8;
 mWriteByte(0x33, thresholdHigh);

    uint8_t thresholdLow = threshold & 0x00FF;
 mWriteByte(0x32, thresholdLow);
}

uint8_t getMagIntSrc()
{
 uint8_t intSrc = mReadByte(0x31);


 if (intSrc & (1<<0))
 {
  return (intSrc & 0xFE);
 }

 return 0;
}

void sleepGyro(_Bool enable)
{
 uint8_t temp = xgReadByte(0x23);
 if (enable) temp |= (1<<6);
 else temp &= ~(1<<6);
 xgWriteByte(0x23, temp);
}

void enableFIFO(_Bool enable)
{
 uint8_t temp = xgReadByte(0x23);
 if (enable) temp |= (1<<1);
 else temp &= ~(1<<1);
 xgWriteByte(0x23, temp);
}

void setFIFO(enum fifoMode_type fifoMode, uint8_t fifoThs)
{


 uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
 xgWriteByte(0x2E, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t getFIFOSamples()
{
 return (xgReadByte(0x2F) & 0x3F);
}

void constrainScales()
{
 if ((IMUSettings.gyro.scale != 245) && (IMUSettings.gyro.scale != 500) &&
  (IMUSettings.gyro.scale != 2000))
 {
  IMUSettings.gyro.scale = 245;
 }

 if ((IMUSettings.accel.scale != 2) && (IMUSettings.accel.scale != 4) &&
  (IMUSettings.accel.scale != 8) && (IMUSettings.accel.scale != 16))
 {
  IMUSettings.accel.scale = 2;
 }

 if ((IMUSettings.mag.scale != 4) && (IMUSettings.mag.scale != 8) &&
  (IMUSettings.mag.scale != 12) && (IMUSettings.mag.scale != 16))
 {
  IMUSettings.mag.scale = 4;
 }
}

void xgWriteByte(uint8_t subAddress, uint8_t data)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  LSM9DS1_I2CwriteByte(_xgAddress, subAddress, data);


}

void mWriteByte(uint8_t subAddress, uint8_t data)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  return LSM9DS1_I2CwriteByte(_mAddress, subAddress, data);


}

uint8_t xgReadByte(uint8_t subAddress)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  return LSM9DS1_I2CreadByte(_xgAddress, subAddress);


 return -1;
}

uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  return LSM9DS1_I2CreadBytes(_xgAddress, subAddress, dest, count);


 return -1;
}

uint8_t mReadByte(uint8_t subAddress)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  return LSM9DS1_I2CreadByte(_mAddress, subAddress);


 return -1;
}

uint8_t mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{


 if (IMUSettings.device.commInterface == IMU_MODE_I2C)
  return LSM9DS1_I2CreadBytes(_mAddress, subAddress, dest, count);


 return -1;
}
# 1205 "SparkFunLSM9DS1.c"
void LSM9DS1_I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    I2C1_Write1ByteRegister(address, subAddress, data);





}

uint8_t LSM9DS1_I2CreadByte(uint8_t address, uint8_t subAddress)
{
 uint8_t data;







    data = I2C1_Read1ByteRegister(address, subAddress);
 return data;
}

uint8_t LSM9DS1_I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
# 1245 "SparkFunLSM9DS1.c"
    I2C1_ReadDataBlock(address, subAddress, dest, count);

 return count;
}
