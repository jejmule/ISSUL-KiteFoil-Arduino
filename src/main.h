//***************************************************** IMU declarations **************************************************
#include <Arduino_LSM9DS1.h>
void readIMU(float *acc, float *gyro, float *mag);
void setNAN(float *array);

//***************************************************** LOADS CELLS declarations **************************************************
#include "HX711.h"
#define CELLS 6   //TOTALS CELLS on setup
#define CELLS_CONNECTED 6   //CELLS connected 
#define SIZE 2   //Data packet per BLE transmission
#define GAIN_WEIGHT 1960  //Weigth use for factory calibration
#define V_EXCITATION 5    //Load cell excitation voltage
#define CALIBRATION 0     //enter calibration mode
//Data and clock pins
const uint8_t data_pin[6] = {D2,D4,D6,D8,PIN_A1,PIN_A3};
const uint8_t clock_pin[6] = {D3,D5,D7,D9,PIN_A2,PIN_A4};
//factory calibration gain of each load_cells
const float gain[6] = {1.05247,1.14560,1.11850,1.12165,1.16922,1.14483};
//Array of HX711 load cells
HX711 load_cell[CELLS]; 
//fonction to handle with load cells
void initLoadCells();
void tareloadCells(int iteration);
void powerLoadCells(boolean power);
unsigned long readLoadCells(float *loads);

//***************************************************** BLE data *********************************************************
//custom union and structure type to help data manipzulation and transmission to BLE.

#define DATA_SIZE 4*(1+CELLS+9)

struct data_struct {
    unsigned long t;    //time in micros since start up
    //unsigned long id;   //frame id
    float load[CELLS];  //weight value for each load cell
    float acc[3];       //X,Y,Z acceleration
    float gyro[3];      //X,Y,Z gyroscope
    float mag[3];       //X,Y,Z magnetic field
  };

union {
  data_struct data[SIZE];              //n packet de data 
  byte frame[SIZE*DATA_SIZE];      //4 bytes for unsigned long x2, CELLS * 4 bytes for float array, 9 bytes for IMU
} ble;

//***************************************************** BLE declarations **************************************************
#include <ArduinoBLE.h>
BLEService kiteService("924c4b43-75b0-48d8-9255-32c4b6e09bee");//GATT custom service definition
BLEService uploadService("73c62c88-02f5-4d2b-a824-e748e9aaec26");//GATT custom service definition

//Standard GATT Characteristic for battery
BLEUnsignedCharCharacteristic battChar("2A19",BLERead | BLENotify);
//Custom GATT Characteristic for internal measurement
BLECharacteristic dataChar("3fd2a1ce-2f24-4c33-b28f-639775d9df43",BLERead | BLENotify, SIZE*DATA_SIZE);
//Custom GATT Characteristic to tare load cells
BLEUnsignedCharCharacteristic tareChar("743e6a48-03f8-4511-ab7a-6caa052ffdaf",BLEWrite);
//Custom GATT Characteristic to start recording on SD card
BLEBooleanCharacteristic recordChar("0d55dc41-33c6-4a43-888c-989f7911420f",BLEWrite);
//Custom GATT Characteristic to upload data file to phone
BLECharacteristic uploadChar("fd0e2894-e3f2-11eb-ba80-0242ac130004",BLERead | BLENotify, SIZE*DATA_SIZE+1); //one byte added for status 0 last frame sent 1 next frame are coming

void initBLE();
//BLE callback
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void dataCharSubscribed(BLEDevice central, BLECharacteristic characteristic);
void dataCharUnsubscribed(BLEDevice central, BLECharacteristic characteristic);
void dataCharRead(BLEDevice central, BLECharacteristic characteristic);
void tareCharWritten(BLEDevice central, BLECharacteristic characteristic);
void battCharSubscribed(BLEDevice central, BLECharacteristic characteristic);
void recordCharWritten(BLEDevice central, BLECharacteristic characteristic);
//void uploadCharRead(BLEDevice central, BLECharacteristic characteristic);

//****************************************************SD CARD declarations *************************************
//#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"

// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
//#define SPI_SPEED SD_SCK_MHZ(50)

#define SD_FAT_TYPE 1
// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
typedef File file_t;
#elif SD_FAT_TYPE == 1
SdFat32 SD;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
SdExFat SD;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
SdFs sd;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

//LED control
//switch on RED and GREEN to obtain YELLOW
//LED color code
//GREEN continous : connected to application
//YELLOW continous : init succesfull waiting for bluetooth connction
//RED continous : init failed, blocked in a while loop -> also during tare
//RED blinking : recording to SD
//YELLOW blinking : uploading file

#define RED 0
#define GREEN 1
#define YELLOW 2
void switch_led(bool status, int color);