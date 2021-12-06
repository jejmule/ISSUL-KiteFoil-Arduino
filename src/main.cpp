//Université de Lausanne - ISSUL
//Jerome Parent
//Read 6 HX711 scale and stream data to serial port
//Using https://github.com/RobTillaart/HX711
//https://web.archive.org/web/20170711074819/https://www.bluetooth.com/specifications/gatt/services

#include "main.h"
#define DEBUG 0

//data frame id
int frame = 0;
int last_frame = 0;
int count;

//Battery constante
int batPin = A0;
byte battery_level_previous = 0;
#define N_BATTERY 100
byte battery_level = 0;
int battery_analog = 0;
byte battery_reading = 0;

//record

boolean record = false;
boolean uploading = false;
String filePrefix = "log_";
String fileName;
file_t dataFile;
byte upload_buffer[SIZE*DATA_SIZE+1];
#define BUFFER 512
byte buffer[BUFFER];
int buffer_position = 0;
int byte_written_SD = 0;

//**************************************************** TIMER *************************************
unsigned long microsNow = 0;

//Timer battery
unsigned long   millisPreviousBattery = 0;
unsigned long   millisPeriodBattery = 100;

//Timer data
unsigned long   microsPeriodData = 1000000 / 80;  //80hz is the maximum rate of load cells 
unsigned long   microsPreviousData = 0;

//Timer upload
unsigned long   microsPeriodUpload = 1000000 / 50;  //period
unsigned long   microsPreviousUpload = 0;

//***************************************************** SETUP **************************************************
void setup() {
  if(DEBUG) delay(10000);
  Serial.println("Beginning of setup");
  // Init serial communication
  Serial.begin(9600);

  //INIT LED
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin 
  pinMode(A6, OUTPUT); // initialize RED_LED
  pinMode(A7, OUTPUT); // initialize GREEN_LED

  
  //Init load_cells
  initLoadCells();
  //power up load cells
  powerLoadCells(true);

  //INIT inertial measurement unit : IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    switch_led(true,RED);
    while (1);
  }
  
  //INIT BLE
  //initBLE();
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    switch_led(true,RED);
    while (1);
  }

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  //Add service and characteritics
  BLE.setLocalName("KiteBoardSensing");
  BLE.setAdvertisedService(kiteService); // add the service UUID
  kiteService.addCharacteristic(dataChar);
  kiteService.addCharacteristic(battChar);
  kiteService.addCharacteristic(tareChar);
  kiteService.addCharacteristic(recordChar);
  kiteService.addCharacteristic(uploadChar);
  BLE.addService(kiteService);

  // assign notification for dataChar
  dataChar.setEventHandler(BLESubscribed, dataCharSubscribed);
  dataChar.setEventHandler(BLEUnsubscribed, dataCharUnsubscribed);
  tareChar.setEventHandler(BLEWritten,tareCharWritten);
  battChar.setEventHandler(BLESubscribed, battCharSubscribed);
  recordChar.setEventHandler(BLEWritten,recordCharWritten);

  //Start advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  //BATTERY MONITORING
  analogReference(AR_VDD);
  analogReadResolution(12);

  //INIT SD
  if (!SD.begin())  {
    Serial.println("SD initialization failed.");
    switch_led(true,RED);
    while (1);
  }
  else
    Serial.println("SD initialization done.");

  //Switch LED to YELLOW
  switch_led(true,YELLOW);
}

//***************************************************** MAIN LOOP **************************************************
void loop() {

  microsNow = micros();
  BLE.poll(1);

  /* if(!BLE.connected()) {
    Serial.println("not connected");
  } */

  //upload data
  if (uploading && (abs(microsNow-microsPreviousUpload) >= microsPeriodUpload)) {
      if(dataFile) {
        int result = dataFile.read(upload_buffer+1, SIZE*DATA_SIZE);
        //result is the number of bytes read from file
        if(result <= 0) {
          upload_buffer[0] = 0; //last frame
          //close data file
          dataFile.close();
          uploading = false;
          Serial.println("Upload completed");
          switch_led(true,GREEN);
        }
        else {
          upload_buffer[0] = 1; //current frame
          //change color at each sent data frame
          switch_led(frame%3<1,YELLOW);
        }
        //write value to upload
        uploadChar.writeValue(upload_buffer,SIZE*DATA_SIZE+1);
        frame++;
      }
      microsPreviousUpload = microsNow;
  }

  //read and stream/record data when we are not uploading file
  if (!uploading && (abs(microsNow - microsPreviousData) >= microsPeriodData)) {
    //add timestamps and id to ble 
    ble.data[count].t = microsNow;
    //ble.data[count].id = frame;
    //READ DATA
    readLoadCells(ble.data[count].load);
    readIMU(ble.data[count].acc, ble.data[count].gyro, ble.data[count].mag);
    if(record) switch_led(frame%9<1,RED);
    //increment frame and update timer
    frame++;
    microsPreviousData = microsNow;
  }

  count = frame % SIZE;
  if(!uploading && frame>last_frame && frame >0 && count == 0) {
    //write to data ble if not reccording
    if(dataChar.subscribed()) dataChar.writeValue(ble.frame,SIZE*DATA_SIZE);
    //record to sdcard
    if(record && dataFile) {
      //unsigned long before = micros();
      byte_written_SD += dataFile.write(ble.frame,SIZE*DATA_SIZE);
      //unsigned long after = micros();
      //Serial.println("write to in "+String(after-before)+" us");
    }
    last_frame = frame;
  }

  if (byte_written_SD >= 2*SIZE*DATA_SIZE) {
    //dataFile.flush();
    //byte empty[BUFFER-2*SIZE*DATA_SIZE];
    //dataFile.write(empty,BUFFER-2*SIZE*DATA_SIZE);
    byte_written_SD = 0;
  }

  // update battery level information
  unsigned long millisNow = millis();
  if (abs(millisNow-millisPreviousBattery) >= millisPeriodBattery) {
    //battery is computed of N_BATTERY measurement to average reading noise
    if(battery_reading % N_BATTERY == 0) {
      battery_analog /= (N_BATTERY);
      //read battery value. The LIPO battery is connected via a resitive diviser bridge with a factor2
      //3300 mV is maximum value read on nano board. divided by 4095 since it is a 12 bits read by default
      int V_batt = 2*3300*battery_analog/4095;
      if(DEBUG) Serial.println("Battery voltage : "+String(V_batt)+" mV");
      //LIPO voltage min is 3000 and max is 4200 -> usuable range is scall between min and max voltage
      battery_level = (V_batt-3000)*100/1200;
      //send battery level to ble
      if(battChar.subscribed() && abs(battery_level-battery_level_previous)>1) {
        Serial.println("Battery level updated : "+String(battery_level)+" %");
        battChar.writeValue(battery_level);
        battery_level_previous = battery_level;
      }
      //reset battery variable
      battery_reading = battery_analog = 0;
    }
    //read battery level
    battery_analog += analogRead(batPin);
    //increment reading
    battery_reading += 1;
    millisPreviousBattery = millisNow;
  }
}

//***************************************************** LOAD CELLS **************************************************
void initLoadCells() {
  for(byte i=0; i<CELLS_CONNECTED; i++) {
    //define load cells pinout
    load_cell[i].begin(data_pin[i],clock_pin[i]);
    //wait for load cell to be ready for data retrivial
    while (!load_cell[i].is_ready()){
      Serial.println("Wait for load cell "+String(i)+" to be ready");
      switch_led(true,RED);
    }
    switch_led(false,RED);
    Serial.print("Load cell ");
    Serial.print(i);
    Serial.println(" initialized");
    //Compute conversion convertion factor using load cell factory calibration values
    //default gain is 128 (+-20mV) and the analog converter is 24 bits
    float conv = (gain[i]*V_EXCITATION) / (40/pow(2,24) * GAIN_WEIGHT) ;
    //Set scale and offset according to HX711 library
    load_cell[i].set_scale(conv);
    //load_cell[i].tare(10);
  }
}

void tareLoadCells(byte iteration) {
  for(byte i=0; i<CELLS_CONNECTED; i++) {
    //if iteration is >0 tare load cell with n iterations
    if(iteration > 0) load_cell[i].tare(iteration);
    //if iterations is 0 -> reset tare
    else load_cell[i].set_offset(0);
  }
}

//***************************************************** READ LOAD CELLS **************************************************
unsigned long readLoadCells(float *data) {
  for(byte i=0; i<CELLS_CONNECTED; i++) {
       if(CALIBRATION) {
         //read raw data without calibration and tare times CALIBRARTION
         Serial.println("load cell"+String(i)+" "+String(load_cell[i].read_average(CALIBRATION)));
       }
       else {
         //read load_cells with offset and scale applied
         data[i] = load_cell[i].get_units(1);
         //if(DEBUG) Serial.println("Load"+String(i)+" "+String(data[i])+" N");
       }
   }
   return micros();
}

void powerLoadCells(boolean power) {
  for(byte i=0; i<CELLS_CONNECTED; i++) {
    //read load_cells with offset and scale applied
    if(power) load_cell[i].power_up();
    else      load_cell[i].power_down();
  }
}

//*************************************************** READ IMU *************************************************************
void readIMU(float *acc, float *gyro, float *mag) {
  //ACCELERATION
  if (IMU.accelerationAvailable())  IMU.readAcceleration(acc[0], acc[1], acc[2]);
  else  setNAN(acc);
  //GYROSCOPE
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gyro[0], gyro[1], gyro[2]);
  else  setNAN(gyro);
  //MAGNETIC FIELD refresh @ 20Hz
  if (IMU.magneticFieldAvailable()) IMU.readMagneticField(mag[0], mag[1], mag[2]);
  else setNAN(mag);
}

void setNAN(float *array) {
  array[0] = NAN;
  array[1] = NAN;
  array[2] = NAN;
}

//*************************************************** BLE STUFF *************************************************************
void initBLE(){
  return;
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH); // turn on the LED
  switch_led(true,GREEN);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  central.disconnect();
  digitalWrite(LED_BUILTIN, LOW); // turn on the LED
  switch_led(true,YELLOW);
  //Serial.println("reset of the board: ");
  //reset the board on disconnect, help to reconnect when leaving android app.
  //NVIC_SystemReset(); //work around to https://github.com/arduino-libraries/ArduinoBLE/issues/33s
}

void dataCharSubscribed(BLEDevice central, BLECharacteristic characteristic) {
  Serial.println("Data characteristic event subscribed : ");
}
void dataCharUnsubscribed(BLEDevice central, BLECharacteristic characteristic){
  Serial.println("Data characteristic event unsubscribed : ");
}

void tareCharWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("Tare characteristic event, tare written: ");
  byte iter = tareChar.value();
  Serial.println(iter);
  switch_led(true,RED);
  tareLoadCells(iter);
  switch_led(true,GREEN);
}

void battCharSubscribed(BLEDevice central, BLECharacteristic characteristic) {
  Serial.println("Battery characteristic event subscribed : ");
  //force to update the battery level
  battChar.writeValue(battery_level);
  Serial.println("Battery level updated : "+String(battery_level)+" %");
}

void recordCharWritten(BLEDevice central, BLECharacteristic characteristic){
  Serial.print("Record characteristic event, record written: ");
  record = recordChar.value();
  Serial.println(record);
  if(record) {
    int index = 0;
    while(true) {
      char num[4];
      sprintf(num,"%04d",index);
      fileName = filePrefix+String(num)+String(".bin");
      if (!SD.exists(fileName)) {
        dataFile = SD.open(fileName,  O_WRITE | O_CREAT);
        Serial.println("Data file created : "+fileName);
        break;
      }
      else index += 1;
    }
    //send filename name on upload char
    Serial.println("Send filename to central");
    upload_buffer[0] = 2;
    fileName.getBytes(upload_buffer+1,12+1); //copy filename to upload buffer, 8 char + . + 3 chr for extension since the filename is 8.3 filename standard + 1 bytes for trailing 0
    uploadChar.writeValue(upload_buffer,1+12+1);
    Serial.println("Start recording on SD card");
    //reset frame number
    frame = last_frame = 0;
  }
  else {
    Serial.println("Stop Recording and closing file");
    dataFile.close();
    dataFile = SD.open(fileName, FILE_READ);
    
    Serial.println("File "+fileName+" open for upload "+dataFile);
    uploading = true;
    frame = 0;
    //dataFile.close();
    //Serial.println("data file closed");
  }
}

//LED CONTROL

void switch_led(bool state,int color=0) {
  //variable for intensity
  int int_red = 0;
  int int_green = 0;
  //define intensity depending on color
  if(color == RED) {
    int_red = state ? 255 : 0; //max if state true
    int_green = 0; //min
  }
  if(color == GREEN) {
    int_red = 0; //min
    int_green = state ? 255 : 0; //max if state true
  }
  //Red is more powerfull than green, half power is used on red to make a Yellow
  if(color == YELLOW){
    int_red = state ? 128 : 0; //half power if state true
    int_green = state ? 255 : 0; //max if state true
  }
  //A6 : RED, A7 : GREEN
  analogWrite(A6,int_red);
  analogWrite(A7,int_green);
}