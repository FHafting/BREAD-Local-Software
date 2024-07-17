/* Function layouts
Get thermocouple data:        void RLHTRequestThermo(int address, float* t1, float* t2)
Set heating setpoint:         void RLHTCommandSetpoint(int address, byte heater, float heatSetpoint, byte thermocouple, bool enableReverse)
Set heating PID tunings:      void RLHTCommandPID(int address, byte heater, float Kp_set, float Ki_set, float Kd_set)
Set motor speeds:             void DCMTCommandSpeed(int address, byte motor, int speed_set, bool enable)
Set as pump with pH setpoint: void DCMTCommandPH(int address, float pHSetpoint_set, float currentPH_set)
Set as pH PID tunings:        void DCMTCommandPHPID(int address, float Kp_set, float Ki_set, float Kd_set)
Set as turbidity pump:        void DCMTCommandTurbidity(int address, byte motor, byte direction, bool enable, int sample_period)
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

#define SLICE_DATA_INTERVAL_MS 10000

#define NUM_DCMT_SLICES   8
#define NUM_RLHT_SLICES   8
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

DCMT_t DCMT[NUM_DCMT_SLICES];
RLHT_t RLHT[NUM_RLHT_SLICES];
PHDO_t PHDO[NUM_PHDO_SLICES];

int pyro_thermo[9][2] = {   // pyrolyis thermocouple layout {address, thermocouple}
  {16, 2}, // DT
  {17, 1}, // DHT
  {16, 1}, // V
  {15, 2}, // CC
  {15, 1}, // SR
  {14, 1}, // KD
  {10, 1}, // C0
  {11, 1}, // C1
  {12, 1}  // C2
};
float pyro_thermo_val[9] = {0,0,0,0,0,0,0,0,0};   // stores thermocouple values

int pyro_heater[10][4] = {   // pyrolysis heater/fan layout {address, heater, thermocouple, reverseEnable}
  {16, 2, 2, 0}, // DT
  {16, 1, 1, 0}, // V
  {15, 2, 2, 0}, // CC
  {15, 1, 1, 0}, // SR
  {14, 1, 1, 0}, // KD
  {10, 1, 1, 0}, // C0H
  {10, 2, 1, 1}, // C0F
  {11, 1, 1, 0}, // C1H
  {11, 2, 1, 1}, // C1F
  {12, 1, 1, 1}  // C2F
};
float pyro_heater_pid[10][4];  // pyrolysis heater/fan PID {setpoint, Kp, Ki, Kd}

int bio_thermo[2][2] = {    // bioreator thermocouples {address, thermocouple}
  {10, 2},
  {11, 2}
};
int bio_post_heaters[2][4] = {    // bioreactor post processing heaters {address, heater, thermocouple, reverseEnable}
  {12, 2, 2, 0},    // pasteurization
  {14, 2, 2, 0}     // dryer
};
float bio_post_heater_pid[2][4];  // {setpoint, Kp, Ki, Kd}
float bio_thermo_val[4] = {0,0,0,0};

int bio_ph[2][2] = {      // bioreactor pH layout {PHDO address, DCMT address}
  {98, 20},
  {100, 21}
};
float bio_ph_val[2][5];   // bioreactor pH settings (pH, Setpoint, Kp, Ki, Kd)

int bio_do[2] = {97, 99}; // bioreactor DO addresses
float bio_do_val[2] = {0,0};      // bioreactor DO values

int bio_turbidity[2][4] = {   // bioreactor turbidity pump {address, motor, direction, sampleTime}
  {22, 1, 1, 0},
  {22, 2, 1, 0}
};
float bio_turbidity_val[2] = {0,0};   // bioreactor turbidity value

int bio_motors[10][3] = {     // bioreactor motor config {address, motor, speed}
  {23, 1, 0},   // stirring 1
  {20, 1, 0},   // base 1
  {20, 2, 0},   // acid 1
  {24, 1, 0},   // water
  {24, 2, 0},   // spent media
  {23, 2, 0},   // stirring 2
  {21, 1, 0},   // base 2
  {21, 2, 0},   // acid 2
  {25, 1, 0},   // harvest
  {25, 2, 0}    // TFF
};

int chem_thermo[2][4] = {   // {address, heater, thermocouple, reverseEnable}
  {13, 1, 1, 0},    // reactor 1
  {13, 2, 2, 0}     // reactor 2
};
float chem_thermo_val[2] = {0,0};
float chem_heater_pid[2][4];

int chem_motors[4][3] = {
  {26, 1, 0},   // ammonium hydroxide
  {26, 2, 0},   // liquid transfer
  {27, 1, 0},   // water dilute
  {27, 2, 0}    // reactor wash
};

AsyncWebServer server(80);

AsyncEventSource events("/events");

bool logging = false; //logging data to SD card and graphs

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

void removeFile(fs::FS &fs, const char * path){
  Serial.printf("Removing file: %s\n", path);

  bool file = fs.remove(path);
  if(!file){
    Serial.println("Failed to remove file");
    return;
  } else {
    Serial.println("Removed file");
  }
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

/*
Pyrolysis Commands:
  ps1:  pyrolysis setpoint 1
  pp1:  pyrolysis Kp 1
  pi1:  pyrolysis Ki 1
  pd1:  pyrolysis Kd 1

Bioreactor Commands:
  bs1:  bioreactor setpoint 1
  bp1:  bioreactor Kp 1
  bi1:  bioreactor Ki 1
  bd1:  bioreactor Kd 1
  bm1:  bioreactor motor 1
  bt1:  bioreactor turbidity sample time 1

Chem Decon Commands:
  cm1:  chemreactor motor 1 (pump)
  cs1:  chemreactor setpoint 1
  cp1:  chemreactor Kp 1
  ci1:  chemreactor Ki 1
  cd1:  chemreactor Kd 1
*/

  server.on("/form-submit", HTTP_POST, [](AsyncWebServerRequest * request) {
    int params = request->params();
    for (int i = 0; i < params; i++) {
      AsyncWebParameter* p = request->getParam(i);
      Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      String postName = p->name().c_str();
      String postValue = p->value().c_str();
      uint8_t index = postName.substring(2).toInt() - 1;

      if(postName.charAt(0) == 'p') {   // pyrolysis reactor
        switch(postName.charAt(1)) {
          case 's': //setpoint
            pyro_heater_pid[index][0] = postValue.toFloat();
            RLHTCommandSetpoint(pyro_heater[index][0], pyro_heater[index][1], pyro_heater_pid[index][0], pyro_heater[index][2], pyro_heater[index][3]);
            break;
          case 'p': //Kp
            pyro_heater_pid[index][1] = postValue.toFloat();
            break;
          case 'i': //Ki
            pyro_heater_pid[index][2] = postValue.toFloat();
            break;
          case 'd': //Kd
            pyro_heater_pid[index][3] = postValue.toFloat();
            RLHTCommandPID(pyro_heater[index][0], pyro_heater[index][1], pyro_heater_pid[index][1], pyro_heater_pid[index][2], pyro_heater_pid[index][3]);
            break;
        }
        //float converted into 4 bytes
      }else if(postName.charAt(0) == 'b') {
        uint8_t index = postName.substring(2).toInt() - 1;
        switch(postName.charAt(1)) {
          case 's': //setpoint
            bio_ph_val[index][1] = postValue.toFloat();
            DCMTCommandPH(bio_ph[index][1], bio_ph_val[index][1], bio_ph_val[index][0]);
            break;
          case 'p': //Kp
            bio_ph_val[index][2] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'i': //Ki
            bio_ph_val[index][3] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'd': //Kd
            bio_ph_val[index][2] = postValue.toFloat();
            Serial.println("Sending pH PID");
            DCMTCommandPHPID(bio_ph[index][1], bio_ph_val[index][2], bio_ph_val[index][3], bio_ph_val[index][4]);
            break;
          case 'm':
            bio_motors[index][2] = postValue.toInt();
            DCMTCommandSpeed(bio_motors[index][0], bio_motors[index][1], bio_motors[index][2], 1);
            break;
          case 't':
            bio_turbidity[index][3] = postValue.toInt();  // sample time
            DCMTCommandTurbidity(bio_turbidity[index][0], bio_turbidity[index][1], bio_turbidity[index][2], 1, bio_turbidity[index][3]);
            break;
        }
      } else if(postName.charAt(0) == 'c') {
        switch(postName.charAt(1)) {
          case 's': //setpoint
            chem_heater_pid[index][0] = postValue.toFloat();
            RLHTCommandSetpoint(chem_thermo[index][0], chem_thermo[index][1], chem_heater_pid[index][0], chem_thermo[index][2], chem_thermo[index][3]);
            //send command to BREAD here:
            break;
          case 'p': //Kp
            chem_heater_pid[index][1] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'i': //Ki
            chem_heater_pid[index][2] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'd': //Kd
            chem_heater_pid[index][3] = postValue.toFloat();
            RLHTCommandPID(chem_thermo[index][0], chem_thermo[index][1], chem_heater_pid[index][1], chem_heater_pid[index][2], chem_heater_pid[index][3]);
            break;
          case 'm':
            chem_motors[index][2] = postValue.toInt();
            DCMTCommandSpeed(chem_motors[index][0], chem_motors[index][1], chem_motors[index][2], 1);
            break;
        }
      }else if(postName.charAt(0) == 'P') {   // bioreactor post processing
        uint8_t index = postName.substring(2).toInt() - 1;
        switch(postName.charAt(1)) {
          case 's': //setpoint
            bio_post_heater_pid[index][0] = postValue.toFloat();
            RLHTCommandSetpoint(bio_post_heaters[index][0], bio_post_heaters[index][1], bio_post_heater_pid[index][0], bio_post_heaters[index][2], bio_post_heaters[index][3]);
            break;
          case 'p': //Kp
            bio_post_heater_pid[index][1] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'i': //Ki
            bio_post_heater_pid[index][2] = postValue.toFloat();
            //send command to BREAD here:
            break;
          case 'd': //Kd
            bio_post_heater_pid[index][3] = postValue.toFloat();
            Serial.println("Sending pH PID");
            RLHTCommandPID(bio_post_heaters[index][0], bio_post_heaters[index][1], bio_post_heater_pid[index][1], bio_post_heater_pid[index][2], bio_post_heater_pid[index][3]);
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
           
      for(uint8_t x = 0; x < 10; x++) {
        toServer += (
          String(pyro_heater_pid[x][0]) + "|" +   // setpoint
          String(pyro_heater_pid[x][1]) + "|" +          // Kp
          String(pyro_heater_pid[x][2]) + "|" +          // Ki
          String(pyro_heater_pid[x][3]) + ","            // Kd
        );
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';

      for(uint8_t x = 0; x < 2; x++) {
        toServer += (
          String(bio_ph_val[x][1]) + "|" +    // setpoint
          String(bio_ph_val[x][2]) + "|" +    // Kp
          String(bio_ph_val[x][3]) + "|" +    // Ki
          String(bio_ph_val[x][4]) + ","      // Kd
        );
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';
      toServer += String(bio_turbidity[0][3]) + "," + String(bio_turbidity[1][3]);
      toServer += '-';

      for(uint8_t x = 0; x < 2; x++) {
        toServer += (
          String(bio_post_heater_pid[x][0]) + "|" +    // setpoint
          String(bio_post_heater_pid[x][1]) + "|" +    // Kp
          String(bio_post_heater_pid[x][2]) + "|" +    // Ki
          String(bio_post_heater_pid[x][3]) + ","      // Kd
        );
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';
      
      for(uint8_t x = 0; x < 10; x++) {
        toServer += String(bio_motors[x][2]) + ",";   // motor speed
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';

      for(uint8_t x = 0; x < 4; x++) {
        toServer += String(chem_motors[x][2]) + ",";  // motor speed
      }
      toServer.remove(toServer.length()-1);
      toServer += '-';

      for(uint8_t x = 0; x < 2; x++) {
        toServer += (
          String(chem_heater_pid[x][0]) + "|" +    // setpoint
          String(chem_heater_pid[x][1]) + "|" +    // Kp
          String(chem_heater_pid[x][2]) + "|" +    // Ki
          String(chem_heater_pid[x][3]) + ","      // Kd
        );
      }
      toServer.remove(toServer.length()-1);
      
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
      writeFile(SD, "/pyrolysis-data.csv", "Date and Time,Dissolution Tank,Dissolution Heating Tape,Valve,Char Chamber,Secondary Reactor,Knockout Drum,Condenser 0,Condenser 1,Condenser 2");
    } else if(request->url() == "/delete-bioreactor") {
      writeFile(SD, "/bioreactor-data.csv", "Date and Time,Thermocouple 1,pH Sensor 1,Dissolved Oxygen 1,Turbidity 1,Thermocouple 2, pH Sensor 2, Dissolved Oxygen 2,Turbidity 2,Pasteurization,Dryer");
    } else if(request->url() == "/delete-chemreactor") {
      writeFile(SD, "/chemreactor-data.csv", "Date and Time,Reactor 1,Reactor 2");
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

uint8_t loggingCounter = 6;
void loop() {
  //get slice data from slices
  if(millis() - lastPOST > SLICE_DATA_INTERVAL_MS) {
    Serial.println("getting data");
    //get time
    String timeToServer = rtc.getTime("%F %T");
    String pyrolysisToServer = timeToServer;
    String bioToServer = timeToServer;
    String chemToServer = timeToServer;
    
    //Heating data
    for(int i=0;i<9;i++){
      float temp;
      if(pyro_thermo[i][1] == 1)
        RLHTRequestThermo(pyro_thermo[i][0], &(pyro_thermo_val[i]), &(temp));
      if(pyro_thermo[i][1] == 2)
        RLHTRequestThermo(pyro_thermo[i][0], &(temp), &(pyro_thermo_val[i]));
    }

    //Bioreactor data

    for(uint8_t n = 0; n < 2; n++) {
      if(readRequestedPHDO) bio_ph_val[n][0] = PHDORequest(bio_ph[n][0]);
      if(readRequestedPHDO) bio_do_val[n] = PHDORequest(bio_do[n]);
      DCMTRequestTurbidity(bio_turbidity[n][0], &(bio_turbidity_val[0]), &(bio_turbidity_val[1]));

      // update pH pump controller with new pH values if setpoint is nonzero
      if(bio_ph_val[n][1] != 0)
        DCMTCommandPH(10 + n, bio_ph_val[n][1], bio_ph_val[n][0]);

      float temp;
      if(bio_thermo[n][1] == 1)
        RLHTRequestThermo(bio_thermo[n][0], &(bio_thermo_val[n]), &(temp));
      if(bio_thermo[n][1] == 2)
        RLHTRequestThermo(bio_thermo[n][0], &(temp), &(bio_thermo_val[n]));
    }
    for(int i=0;i<2;i++){
      float temp;
      if(bio_post_heaters[i][2] == 1)
        RLHTRequestThermo(bio_post_heaters[i][0], &(bio_thermo_val[i+2]), &(temp));
      if(bio_post_heaters[i][2] == 2)
        RLHTRequestThermo(bio_post_heaters[i][0], &(temp), &(bio_thermo_val[i+2]));
    }
    readRequestedPHDO = false;  // reload new read request
    
    //Chemreactor data
    for(uint8_t n = 0; n < 2; n++) {
      float temp;
      if(chem_thermo[n][2] == 1)
        RLHTRequestThermo(chem_thermo[n][0], &(chem_thermo_val[n]), &(temp));
      if(chem_thermo[n][2] == 2)
        RLHTRequestThermo(chem_thermo[n][0], &(temp), &(chem_thermo_val[n]));
    }

    //convert to data to string to save to SD card and send to server
    for(uint8_t n = 0; n < 9; n++) {
      pyrolysisToServer += "," + String(pyro_thermo_val[n]);
    }
    for(uint8_t n = 0; n < 2; n++) {
      bioToServer += "," + String(bio_thermo_val[n]) + "," + String(bio_ph_val[n][0]) + "," + String(bio_do_val[n]) + "," + String(bio_turbidity_val[n]);
    }
    bioToServer += "," + String(bio_thermo_val[2]) + "," + String(bio_thermo_val[3]);
    for(uint8_t n = 0; n < 2; n++) {
      chemToServer += "," + String(chem_thermo_val[n]);
    }

    //send data to website
/*
Readings to Webpage Format
Pyrolysis Reactor:
  Format:  Date and Time,Dissolution Tank,Dissolution Heating Tape,Valve,Char Chamber,Secondary Reactor,Knockout Drum,Condenser 0,Condenser 1,Condenser 2
  Index:        0       ,       1        ,            2           ,   3 ,     4      ,        5        ,      6      ,     7     ,     8     ,     9    
Bioreactor:
  Format:  Date and Time,Thermo1,pH1,DO1,Turb1,Thermo2,pH2,DO2,Turb2,Past,Dry
  Index:       0        ,   1   , 2 , 3 ,  4  ,   5   , 6 , 7 ,  8  ,  9 , 10
Chemreactor:
  Format:  Date and Time,Reactor 1,Reactor 2
  Index:         0      ,    1    ,    2
*/
    events.send(pyrolysisToServer.c_str(), "pyrolysis-readings", millis());
    events.send(bioToServer.c_str(), "bioreactor-readings", millis());
    events.send(chemToServer.c_str(), "chemreactor-readings", millis());
    
    //log onto the SD card
    if(logging){
      appendFile(SD, "/pyrolysis-data.csv", "\r\n" + pyrolysisToServer);
      appendFile(SD, "/bioreactor-data.csv", "\r\n" + bioToServer);
      appendFile(SD, "/chemreactor-data.csv", "\r\n" + chemToServer);
    }
    lastPOST = millis();
  }else if(!readRequestedPHDO){
    for(uint8_t n = 0; n < 2; n++){
      PHDOCommand(bio_do[n], "r");  // request reading from DO sensors
      PHDOCommand(bio_ph[n][0], "r");  // request reading from pH sensors
    }
    readRequestedPHDO = true;
  }
}

void RLHTRequestThermo(int address, float* t1, float* t2)
{
  bool data_received = false;
  byte in_char;
  char in_data[20];
  FLOATUNION_t thermo1;
  FLOATUNION_t thermo2;

  thermo1.number = 0;
  thermo2.number = 0;
  
  Wire.requestFrom(address, 8, 1);
  int i=0;
  while (Wire.available()) {                 //are there bytes to receive.
    data_received = true;
    in_char = Wire.read();                   //receive a byte.
    in_data[i] = in_char;                    //load this byte into our array.
    i++;                                  //incur the counter for the array element.
  }

  if(data_received){
    for(int x=0;x<4;x++)
    {
      thermo1.bytes[x] = in_data[x];
      thermo2.bytes[x] = in_data[x+4];
    }
  }

  *t1 = thermo1.number;
  *t2 = thermo2.number;
}

void RLHTCommandSetpoint(int address, byte heater, float heatSetpoint, byte thermocouple, bool enableReverse)
{
  FLOATUNION_t setpoint;
  setpoint.number = heatSetpoint;

  Wire.beginTransmission(address);
  Wire.write('H');
  Wire.write(heater);
  for(int i=0; i<4; i++){
    Wire.write(setpoint.bytes[i]);              // sends one byte
  }
  Wire.write(thermocouple);
  Wire.write(enableReverse);
  Wire.endTransmission();    // stop transmitting

  Serial.print(address);
  Serial.print('H');
  Serial.print(heater);
  Serial.print(setpoint.number);
  Serial.print(thermocouple);
  Serial.println(enableReverse);
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

void DCMTRequestTurbidity(int address, float* turbidity1, float* turbidity2)
{
  bool data_received = false;
  byte in_char;
  char in_data[20];
  FLOATUNION_t turb1, turb2;

  turb1.number = 0;
  turb2.number = 0;
  
  Wire.requestFrom(address, 10, 1);
  int i=0;
  while (Wire.available()) {                 //are there bytes to receive.
    data_received = true;
    in_char = Wire.read();                   //receive a byte.
    in_data[i] = in_char;                    //load this byte into our array.
    i++;                                  //incur the counter for the array element.
  }

  if(data_received){
    for(int x=0;x<4;x++){
      turb1.bytes[x] = in_data[x];
      turb2.bytes[x] = in_data[x+4];
    }
  }

  *turbidity1 = turb1.number;
  *turbidity2 = turb2.number;
}

void DCMTCommandTurbidity(int address, byte motor, byte direction, bool enable, int sample_period)
{
  INTUNION_t sample_p;
  sample_p.number = sample_period;

  Wire.beginTransmission(address);
  Wire.write('T');
  Wire.write(motor);
  Wire.write(direction);
  Wire.write(enable);
  for(int i=0;i<4;i++) Wire.write(sample_p.bytes[i]);
  Wire.endTransmission();
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
  if(code == 1){
    while (Wire.available()) {                 //are there bytes to receive.
      in_char = Wire.read();                   //receive a byte.
      if(code != 1)                            // check if incoming data is valid
        in_data[i] = 0;                        // set values to zero if not valid data
      else
        in_data[i] = in_char;                    //load this byte into our array.
      i++;                                  //incur the counter for the array element.
      if (in_char == 0) {                      //if we see that we have been sent a null command.
        i = 0;                                 //reset the counter i to 0.
        break;                                 //exit the while loop.
      }
    }
    return atof(in_data);
  }else
    return 0;
}