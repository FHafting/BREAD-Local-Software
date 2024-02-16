#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <ArduinoJson.h>

#include "SD.h"
#include "FS.h"
#include "SPI.h"

#include <ESP32Time.h>
ESP32Time rtc;

// microSD Card Reader connections
#define SD_CS          5
#define SPI_MOSI      23 
#define SPI_MISO      19
#define SPI_SCK       18

const char* ssid     = "BREAD-DARPA";
const char* password = "12345678";

AsyncWebServer server(80);

AsyncEventSource events("/events");

bool logging = false; //logging data to SD card and graphs

//slice data variables
#define NUM_OF_DATA 6
uint16_t pyrolysisData[NUM_OF_DATA];

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

void setup() {
  Serial.begin(115200);
  
  initSDCard();

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
    } else if(request->url() == "/get-variables") {
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
      
      if(logging) {
        request->send(200, "text/plain", "logging");
      } else {
        request->send(200, "text/plain", "not logging");
      }      
    } else if(request->url() == "/logging") {
      Serial.println("log to SD card");
      logging = true;
    } else if(request->url() == "/not-logging") {
      Serial.println("stop log to SD card");
      logging = false;
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
}

uint32_t lastPOST = 0;
void loop() {
  //get slice data from slices
  if(millis() - lastPOST > 2000) {
    Serial.println("getting data");
    //get time
    String toServer = rtc.getTime("%F %T");
    
    //receive data from BREAD (currently random numbers)
    for(uint8_t n = 0; n < NUM_OF_DATA; n++) {
      pyrolysisData[n] = random(400);
    }

    //convert to data to string to save to SD card
    for(uint8_t n = 0; n < NUM_OF_DATA; n++) {
      toServer += "," + String(pyrolysisData[n]);
    }

    //send data to website
    events.send(toServer.c_str(), "thermocouple-readings", millis());
    
    //log onto the SD card
    if(logging) {
      appendFile(SD, "/data.csv", "\r\n" + toServer);
    }
    lastPOST = millis();
  }
}
