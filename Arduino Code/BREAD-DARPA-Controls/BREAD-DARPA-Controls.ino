/* Function layouts
Get thermocouple data:        void RLHTRequestThermo(int address, float* t1, float* t2)
Set heating setpoint:         void RLHTCommandSetpoint(int address, byte heater, float heatSetpoint)
Set heating PID tunings:      void RLHTCommandPID(int address, byte heater, float Kp_set, float Ki_set, float Kd_set)
Set motor speeds:             void DCMTCommandSpeed(int address, byte motor, int speed_set, bool enable)
Set as pump with pH setpoint: void DCMTCommandPH(int address, float pHSetpoint_set, float currentPH_set)
Set as pH PID tunings:        void DCMTCommandPHPID(int address, float Kp_set, float Ki_set, float Kd_set)
Send commands to pH & DO:     void PHDOCommand(int address, char *commandData)
Get pH or DO readings:        float PHDORequest(int address)
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "SD.h"
#include "FS.h"
#include "SPI.h"
#include <Wire.h>

#include <ESP32Time.h>
ESP32Time rtc;

#define NUM_DCMT_SLICES   7
#define NUM_RLHT_SLICES   6
#define NUM_PHDO_SLICES   2

// microSD Card Reader connections
#define SD_CS          5
#define SPI_MOSI      23 
#define SPI_MISO      19
#define SPI_SCK       18

#define ESTOP   32

const char* ssid     = "BREAD-DARPA";
const char* password = "12345678";

// Useful unions for breaking data into bytes
union FLOATUNION_t //Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
};
union INTUNION_t //Define an int that can be broken up and sent via I2C
{
  int number;
  uint8_t bytes[4];
};

// Slice structures for managing data
struct DCMT_t {
  int address;    // I2C address of Slice
  int m1Speed;   // motor 1 speed (-100-100)
  int m2Speed;   // motor 2 speed (-100-100)
  bool m1En;   // motor 1 enable (0:off/apply brake, 1:on/coast)
  bool m2En;   // motor 2 enable (0:off/apply brake, 1:on/coast)
  float pHSetpoint;
  float currentPH;
  float Kp_pH;
  float Ki_pH;
  float Kd_pH;
};

struct RLHT_t {
  int address;    // I2C address of Slice
  float heatSetpoint_1;   // set desired temperature for heater 1 (if using relay as heater actuator)
  float Kp_1;             // PID proportional gain for heater 1
  float Ki_1;             // PID integral gain for heater 1
  float Kd_1;             // PID derivative gain for heater 1 
  float heatSetpoint_2;
  float Kp_2;
  float Ki_2;
  float Kd_2;
  float thermo1;  // thermocouple 1 measurement
  float thermo2;  // thermocouple 2 measurement
};

struct PHDO_t {
  int addressPH; // I2C address of Atlas Scientific pH EZO board (default = 99)
  int addressDO; // I2C address of Atlas Scientific dissolved oxygen EZO board (default = 97)
  float pH;       // current pH reading
  float DO;       // current dissolved oxygen reading
  bool calPH = false;
  bool calDO = false;
};

DCMT_t DCMT[7];
RLHT_t RLHT[6];
PHDO_t PHDO[2];

AsyncWebServer server(80);

AsyncEventSource events("/events");

bool logging = false; //logging data to SD card and graphs

//slice data variables
#define PYROLYSIS_NUM_DATA 6
float pyrolysisValue[PYROLYSIS_NUM_DATA];
float pyrolysisSetpoint[PYROLYSIS_NUM_DATA];
float pyrolysisKP[PYROLYSIS_NUM_DATA];
float pyrolysisKI[PYROLYSIS_NUM_DATA];
float pyrolysisKD[PYROLYSIS_NUM_DATA];

#define BIO_NUM_DATA 2
#define BIO_NUM_MOTORS 12
float bioThermoData[BIO_NUM_DATA];
float bioPHData[BIO_NUM_DATA];
float bioOxygenData[BIO_NUM_DATA];
float bioPHSetpoint[BIO_NUM_DATA];
float bioPHKP[BIO_NUM_DATA];
float bioPHKI[BIO_NUM_DATA];
float bioPHKD[BIO_NUM_DATA];
float bioMotorSetpoint[BIO_NUM_MOTORS];

float chemPumpSetpoint;
float chemThermoValue;
float chemThermoSetpoint;
float chemThermoKP;
float chemThermoKI;
float chemThermoKD;

void initSlices(){
  // DCMT Slice address assignment
  for(int i=0;i<NUM_DCMT_SLICES;i++){
    DCMT[i].address = 20+i;   // addresses 20-26
  }
  // RLHT Slice address assignment
  for(int i=0;i<NUM_RLHT_SLICES;i++){
    RLHT[i].address = 10+i;   // addresses 10-15
  }
  // addresses of Atlas Scientific Sensors
  PHDO[0].addressDO = 97;
  PHDO[0].addressPH = 98;
  PHDO[1].addressDO = 99;
  PHDO[1].addressPH = 100;
}

void initSDCard(){
  pinMode(SD_CS, OUTPUT);      
  digitalWrite(SD_CS, HIGH); 
    
  // Initialize SPI bus for microSD Card
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  
  if(!SD.begin(SD_CS)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void appendFile(fs::FS &fs, const char * path, String message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

uint32_t lastPOST;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  
  initSDCard();

  initSlices();

  pinMode(ESTOP, OUTPUT);

  //start access point wifi
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("Server started");

  //send html data to server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/index.html", "text/html");
  });

  //load data from slice form
  server.on("/form-submit", HTTP_POST, [](AsyncWebServerRequest * request) {
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      String postName = p->name().c_str();
      String postValue = p->value().c_str();
      if(postName.charAt(0) == 'p') {
        uint8_t index = postName.substring(2).toInt() - 1;
        switch(postName.charAt(1)) {
          case 's': //setpoint
            pyrolysisSetpoint[index] = postValue.toFloat();
            if(index%2 == 0)  // even index
              RLHTCommandSetpoint(RLHT[(index/2)+1].address, 1, pyrolysisSetpoint[index]);
            else    // odd index
              RLHTCommandSetpoint(RLHT[(index/2)+1].address, 2, pyrolysisSetpoint[index]);
            break;
          case 'p': //Kp
            pyrolysisKP[index] = postValue.toFloat();
            break;
          case 'i': //Ki
            pyrolysisKI[index] = postValue.toFloat();
            break;
          case 'd': //Kd
            pyrolysisKD[index] = postValue.toFloat();
            if(index%2 == 0){  // even index
              RLHTCommandPID(RLHT[(index/2)+1].address, 1, pyrolysisKP[index], pyrolysisKI[index], pyrolysisKD[index]);
            }else{    // odd index
              RLHTCommandPID(RLHT[(index/2)+1].address, 2, pyrolysisKP[index], pyrolysisKI[index], pyrolysisKD[index]);
            }
            break;
        }
        //float converted into 4 bytes
      } else if(postName.charAt(0) == 'b') {
        uint8_t index = postName.substring(2).toInt() - 1;
        switch(postName.charAt(1)) {
          case 's': //setpoint
            bioPHSetpoint[index] = postValue.toFloat();
            DCMTCommandPH(DCMT[index+1].address, bioPHSetpoint[index], bioPHData[index]); // send pH 1 to DCMT 21 and pH 2 to DCMT 22
            break;
          case 'p': //Kp
            bioPHKP[index] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'i': //Ki
            bioPHKI[index] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'd': //Kd
            bioPHKD[index] = postValue.toFloat();
            Serial.println("Sending pH PID");
            DCMTCommandPHPID(DCMT[index+1].address, bioPHKP[index], bioPHKI[index], bioPHKD[index]); // pH PID 1 to DCMT 21, pH PID 2 to DCMT 22
            break;
          case 'm':
            bioMotorSetpoint[index] = postValue.toFloat();
            switch(index){
              case 0:   // stirring motor 1
                DCMTCommandSpeed(DCMT[3].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 1:   // sampling pump 1
                DCMTCommandSpeed(DCMT[4].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 2:   // media pump 1
                DCMTCommandSpeed(DCMT[5].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 3:   // acid pump 1
                DCMTCommandSpeed(DCMT[1].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 4:   // base pump 1
                DCMTCommandSpeed(DCMT[1].address, 2, bioMotorSetpoint[index], 1);
                break;
              case 5:   // extra pump 1
                DCMTCommandSpeed(DCMT[6].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 6:   // stirring motor 2
                DCMTCommandSpeed(DCMT[3].address, 2, bioMotorSetpoint[index], 1);
                break;
              case 7:   // sampling pump 2
                DCMTCommandSpeed(DCMT[4].address, 2, bioMotorSetpoint[index], 1);
                break;
              case 8:   // media pump 2
                DCMTCommandSpeed(DCMT[5].address, 2, bioMotorSetpoint[index], 1);
                break;
              case 9:   // acid pump 2
                DCMTCommandSpeed(DCMT[2].address, 1, bioMotorSetpoint[index], 1);
                break;
              case 10:   // base pump 2
                DCMTCommandSpeed(DCMT[2].address, 2, bioMotorSetpoint[index], 1);
                break;
              case 11:   // extra pump 2
                DCMTCommandSpeed(DCMT[6].address, 2, bioMotorSetpoint[index], 1);
                break;
            }
            break;
        }
      } else if(postName.charAt(0) == 'c') {
        switch(postName.charAt(1)) {
          case 's': //setpoint
            chemThermoSetpoint = postValue.toFloat();
            RLHTCommandSetpoint(RLHT[5].address, 1, chemThermoSetpoint);
            //send command to BREAD here:
            break;
          case 'p': //Kp
            chemThermoKP = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'i': //Ki
            chemThermoKI = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'd': //Kd
            chemThermoKD = postValue.toFloat();
            RLHTCommandPID(RLHT[5].address, 1, chemThermoKP, chemThermoKI, chemThermoKD);
            break;
          case 'm':
            chemPumpSetpoint = postValue.toFloat();
            DCMTCommandSpeed(DCMT[0].address, 1, chemPumpSetpoint, 1);
            break;
        }
      }
    }
  });

  //load any non-form POST request body and url
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    Serial.println(request->url());
    if (request->url() == "/all-data") {
      String s = "";
      for(size_t i=0; i<len; i++) {
        s += char(data[i]);
      }
      Serial.println(s);
    }if(request->url() == "/get-variables") {
      //set time on esp32 rtc
      String s = "";
      for(size_t i=0; i<len; i++) {
        s += char(data[i]);
      }
      
      uint16_t timeDate[6];
      for(uint8_t x = 0; x < 5; x++) {
        uint8_t nextComma = s.indexOf(',');
        timeDate[x] = s.substring(0, nextComma).toInt();
        s = s.substring(nextComma+1);
      }
      timeDate[5] = s.toInt();
      rtc.setTime(timeDate[0], timeDate[1], timeDate[2], timeDate[3], timeDate[4], timeDate[5]);

      //how variables are sent to the server (I should probably use a more structured format like JSON)
      String toServer = "";
      if(logging) {
        toServer += "logging-";
      } else {
        toServer += "not logging-";
      }
           
      for(uint8_t x = 0; x < PYROLYSIS_NUM_DATA; x++) {
        toServer += (
          String(pyrolysisSetpoint[x]) + "|" + 
          String(pyrolysisKP[x]) + "|" + 
          String(pyrolysisKI[x]) + "|" + 
          String(pyrolysisKD[x]) + ","
        );
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';

      for(uint8_t x = 0; x < BIO_NUM_DATA; x++) {
        toServer += (
          String(bioPHSetpoint[x]) + "|" +
          String(bioPHKP[x]) + "|" +
          String(bioPHKI[x]) + "|" +
          String(bioPHKD[x]) + ","
        );
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';
      
      for(uint8_t x = 0; x < BIO_NUM_MOTORS; x++) {
        toServer += String(bioMotorSetpoint[x]) + ",";
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';

      toServer += String(chemPumpSetpoint) + ",";
      toServer += String(chemThermoSetpoint) + "|" + String(chemThermoKP) + "|" + String(chemThermoKI) + "|" + String(chemThermoKD);
      
      request->send(200, "text/plain", toServer);
    } if(request->url() == "/logging") {
      Serial.println("log to SD card");
      logging = true;
    } if(request->url() == "/not-logging") {
      Serial.println("stop log to SD card");
      logging = false;
    } if(request->url() == "/estop-on") {
      Serial.println("estop on");
      digitalWrite(ESTOP, HIGH);
    } if(request->url() == "/estop-off") {
      Serial.println("estop off");
      digitalWrite(ESTOP, LOW);
    } else if(request->url() == "/delete-pyrolysis") {
      writeFile(SD, "/pyrolysis-data.csv", "Date and Time,Condenser 1,Condenser 2,Condenser 0,Char Chamber,Dissolution Tank,Valve");
    } else if(request->url() == "/delete-bioreactor") {
      writeFile(SD, "/bioreactor-data.csv", "Date and Time,Thermocouple 1,pH Sensor 1,Dissolved Oxygen 1,Thermocouple 2, pH Sensor 2, Dissolved Oxygen 2");
    } else if(request->url() == "/delete-chemreactor") {
      writeFile(SD, "/chemreactor-data.csv", "Date and Time,Thermocouple");
    }
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.serveStatic("/", SD, "/");
  server.begin();

  lastPOST = millis();
}

uint32_t calTime = 0;
int calDelay = 900;
bool readRequestedPHDO = false;

void loop() {
  //get slice data from slices
  if(millis() - lastPOST > 5000) {
    Serial.println("getting data");
    //get time
    String timeToServer = rtc.getTime("%F %T");
    String pyrolysisToServer = timeToServer;
    String bioToServer = timeToServer;
    String chemToServer = timeToServer;
    
    //Pyrolysis data
    /*
    for(uint8_t n = 0; n < PYROLYSIS_NUM_DATA; n++) { //receive data from BREAD (currently random numbers)
      pyrolysisValue[n] = RLHTRequest;
    }*/
    RLHTRequestThermo(RLHT[1].address, &(pyrolysisValue[0]), &(pyrolysisValue[1]));
    RLHTRequestThermo(RLHT[2].address, &(pyrolysisValue[2]), &(pyrolysisValue[3]));
    RLHTRequestThermo(RLHT[3].address, &(pyrolysisValue[4]), &(pyrolysisValue[5]));

    //Bioreactor data
    for(uint8_t n = 0; n < BIO_NUM_DATA; n++) {
      if(!(PHDO[n].calPH) && readRequestedPHDO) bioPHData[n] = PHDORequest(PHDO[n].addressPH);
      if(!(PHDO[n].calDO) && readRequestedPHDO) bioOxygenData[n] = PHDORequest(PHDO[n].addressDO);
    }
    // update pH pump controller with new pH values if setpoint is nonzero
    if(bioPHSetpoint[0] != 0)
      DCMTCommandPH(DCMT[1].address, bioPHSetpoint[0], bioPHData[0]); // send pH 1 to DCMT 21
    if(bioPHSetpoint[1] != 0)
      DCMTCommandPH(DCMT[2].address, bioPHSetpoint[1], bioPHData[1]); // send pH 2 to DCMT 22
    
    readRequestedPHDO = false;  // reload new read request
    
    RLHTRequestThermo(RLHT[0].address, &(bioThermoData[0]), &(bioThermoData[1]));  // get bioreactor temperature data

    float extraThermo;
    //Chemreactor data
    RLHTRequestThermo(RLHT[5].address, &chemThermoValue, &extraThermo);

    //convert to data to string to save to SD card and send to server
    for(uint8_t n = 0; n < PYROLYSIS_NUM_DATA; n++) {
      pyrolysisToServer += "," + String(pyrolysisValue[n]);
    }
    for(uint8_t n = 0; n < BIO_NUM_DATA; n++) {
      bioToServer += "," + String(bioThermoData[n]) + "," + String(bioPHData[n]) + "," + String(bioOxygenData[n]);
    }
    chemToServer += "," + String(chemThermoValue);

    //send data to website
    events.send(pyrolysisToServer.c_str(), "pyrolysis-readings", millis());
    events.send(bioToServer.c_str(), "bioreactor-readings", millis());
    events.send(chemToServer.c_str(), "chemreactor-readings", millis());
    
    //log onto the SD card
    if(logging) {
      appendFile(SD, "/pyrolysis-data.csv", "\r\n" + pyrolysisToServer);
      appendFile(SD, "/bioreactor-data.csv", "\r\n" + bioToServer);
      appendFile(SD, "/chemreactor-data.csv", "\r\n" + chemToServer);
    }
    lastPOST = millis();
  }else if(!readRequestedPHDO){
    for(uint8_t n = 0; n < BIO_NUM_DATA; n++){
      PHDOCommand(PHDO[n].addressDO, "r");  // request reading from DO sensors
      PHDOCommand(PHDO[n].addressPH, "r");  // request reading from pH sensors
    }
    readRequestedPHDO = true;
  }
}

void RLHTRequestThermo(int address, float* t1, float* t2)
{
  byte in_char;
  char in_data[20];
  FLOATUNION_t thermo1;
  FLOATUNION_t thermo2;
  
  Wire.requestFrom(address, 8, 1);
  int i=0;
  while (Wire.available()) {                 //are there bytes to receive.
    in_char = Wire.read();                   //receive a byte.
    in_data[i] = in_char;                    //load this byte into our array.
    i++;                                  //incur the counter for the array element.
  }

  for(int x=0;x<4;x++)
  {
    thermo1.bytes[x] = in_data[x];
    thermo2.bytes[x] = in_data[x+4];
  }

  *t1 = thermo1.number;
  *t2 = thermo2.number;
}

void RLHTCommandSetpoint(int address, byte heater, float heatSetpoint)
{
  FLOATUNION_t setpoint;
  setpoint.number = heatSetpoint;

  Wire.beginTransmission(address);
  Wire.write('H');
  Wire.write(heater);
  for(int i=0; i<4; i++){
    Wire.write(setpoint.bytes[i]);              // sends one byte
  }
  Wire.endTransmission();    // stop transmitting

  Serial.print(address);
  Serial.print('H');
  Serial.print(heater);
  Serial.println(setpoint.number);
}

void RLHTCommandPID(int address, byte heater, float Kp_set, float Ki_set, float Kd_set)
{
  FLOATUNION_t Kp;
  FLOATUNION_t Ki;
  FLOATUNION_t Kd;

  Kp.number = Kp_set;
  Ki.number = Ki_set;
  Kd.number = Kd_set;

  Wire.beginTransmission(address);
  Wire.write('P');
  Wire.write(heater);
  for(int i=0; i<4; i++){
    Wire.write(Kp.bytes[i]);              // sends one byte
  }
  for(int i=0; i<4; i++){
    Wire.write(Ki.bytes[i]);              // sends one byte
  }
  for(int i=0; i<4; i++){
    Wire.write(Kd.bytes[i]);              // sends one byte
  }
  Wire.endTransmission();    // stop transmitting

  Serial.print(address);
  Serial.print('P');
  Serial.print(heater);
  Serial.print(Kp.number);
  Serial.print(Ki.number);
  Serial.println(Kd.number);
}

void DCMTCommandSpeed(int address, byte motor, int speed_set, bool enable)
{
  INTUNION_t speed;
  speed.number = speed_set;

  Wire.beginTransmission(address);
  Wire.write('M');
  Wire.write(motor);
  for(int i=0; i<4; i++){
    Wire.write(speed.bytes[i]);              // sends one byte
  }
  Wire.write(enable);
  Wire.endTransmission();    // stop transmitting
}

void DCMTCommandPH(int address, float pHSetpoint_set, float currentPH_set)
{
  FLOATUNION_t pHSetpoint, currentPH;
  pHSetpoint.number = pHSetpoint_set;
  currentPH.number = currentPH_set;

  Wire.beginTransmission(address);
  Wire.write('p');
  for(int i=0; i<4; i++) Wire.write(currentPH.bytes[i]);
  for(int i=0; i<4; i++) Wire.write(pHSetpoint.bytes[i]);
  Wire.endTransmission();    // stop transmitting
}

void DCMTCommandPHPID(int address, float Kp_set, float Ki_set, float Kd_set)
{
  FLOATUNION_t Kp, Ki, Kd;
  Kp.number = Kp_set;
  Ki.number = Ki_set;
  Kd.number = Kd_set;

  Wire.beginTransmission(address);
  Wire.write('P');
  for(int i=0; i<4; i++) Wire.write(Kp.bytes[i]);
  for(int i=0; i<4; i++) Wire.write(Ki.bytes[i]);
  for(int i=0; i<4; i++) Wire.write(Kd.bytes[i]);
  Wire.endTransmission();    // stop transmitting
}


void PHDOCommand(int address, char *commandData)
{
  Wire.beginTransmission(address);
  Wire.write(commandData);
  Serial.println(commandData);
  Wire.endTransmission();    // stop transmitting
}

float PHDORequest(int address)
{
  byte in_char;
  char in_data[20];
  FLOATUNION_t data;

  Wire.requestFrom(address, 20, 1);

  byte code = 0;
  code = Wire.read();                                                   //the first byte is the response code, we read this separately.

  switch (code) {                           //switch case based on what the response code is.
    case 1:                                 //decimal 1.
      Serial.println("Success");            //means the command was successful.
      break;                                //exits the switch case.

    case 2:                                 //decimal 2.
      Serial.println("Failed");             //means the command has failed.
      break;                                //exits the switch case.

    case 254:                               //decimal 254.
      Serial.println("Pending");            //means the command has not yet been finished calculating.
      break;                                //exits the switch case.

    case 255:                               //decimal 255.
      Serial.println("No Data");            //means there is no further data to send.
      break;                                //exits the switch case.
  }

  int i=0;
  while (Wire.available()) {                 //are there bytes to receive.
    in_char = Wire.read();                   //receive a byte.
    in_data[i] = in_char;                    //load this byte into our array.
    i++;                                  //incur the counter for the array element.
    if (in_char == 0) {                      //if we see that we have been sent a null command.
      i = 0;                                 //reset the counter i to 0.
      break;                                 //exit the while loop.
    }
  }
  return atof(in_data);
}