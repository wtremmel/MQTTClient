/**
 * MQTTCLient.ino
 * 
 * Reads light and weather sensors and publishes readings via mqtt
 *
 *  Created on: 25.3.2017
 *
 */

//#define ESP1 // Test Arbeitszimmer
// #define ESP2 // Kueche
// #define ESP3 // Garten
#define ESP4 // Test

#if defined(ESP1)
#define OUTDOOR 1
#define BME280ADDR 0x76
#define DEBUG 1
#define LS_FACTOR_M 0x02
#define LSSENSOR
#define UVSENSOR 1
#define ADC 1
#define REGEN_ADC 0

#elif defined(ESP2)
#define INDOOR 1
#define MOTION
#define NROFLEDS 10
#define BME280ADDR 0x77
#define LS_FACTOR_M 0x01

#elif defined(ESP3)
#define OUTDOOR 1
#define BME280ADDR 0x76
#define LS_FACTOR_M 0x02
#define DEBUG 1

#elif defined(ESP4)
#define DEBUG 1
#define OUTDOOR 1
#define NROFLEDS 1
#define TSL2561
#define BME280ADDR 0x76
#define LS_FACTOR_M 0x02
#define ADC 1
#define REGEN_ADC 0 // ADC-Pin 0 for rain detection
#define SOIL_ADC 3 // ADC-Pin 3 for soil moisture
// #define SOIL 1
// #define SOIL_ADC 2 // ADC-Pin 2 for soil moisture
#endif

// 0x39 TSL2561
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor


#include <Arduino.h>

#include <ESP8266WiFi.h>
// #include <ESP8266mDNS.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <FS.h>

#include <SparkFunBME280.h>
#include <ESP8266HTTPClient.h>

// UV-Sensor - just testing at the moment
#ifdef UVSENSOR
#include "Adafruit_VEML6070.h"
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#endif

// Light sensor TSL2561
#ifdef TSL2561
#include <SparkFunTSL2561.h>
SFE_TSL2561 lightTSL2561;
unsigned int msTSL2561;
boolean gainTSL2561;
#endif

#ifdef ADC
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
#endif

//Pin defintions
#define I2CSDA 4  //D2
#define I2CSCL 5  //D1
#define PIRINPUT 12
#define BUZZER 13
#define NEOPIXEL 14 //D5
#define NIKONLED 2 // D4

#ifdef NROFLEDS
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS,NEOPIXEL,NEO_GRB+NEO_KHZ800);
#endif

// Voltage when on power supply
#define VOLTAGE_PS 2700
// Go to longer intervals
#define VOLTAGE_LOW 2030
// Minimum volate for operation
#define VOLTAGE_MIN 2010

// Run modes
#define RM_START 1
#define RM_SENSOR 2
#define RM_CONFIG 4
byte runMode = RM_START | RM_SENSOR | RM_CONFIG;


ADC_MODE(ADC_VCC);

const char* fn_ssid = "/ssid";
const char *fn_pass = "/pass";
const char *fn_myname = "/myname";
const char *fn_mqttserver = "/mqttserver";
const char *fn_site = "/site";
const char *fn_location = "/location";

String Smyname, Spass, Sssid, Smqttserver, Ssite, Slocation;

// Webserver for status and config
WiFiServer server(80);
WiFiClient espClient, webClient;
PubSubClient client;
// experimental
WiFiEventHandler disconnectedEventHandler;



long lastMsg = 0;
char msg[50];
char topic[50];
int value = 0;


int pirInput = PIRINPUT;
int pirState = LOW;

unsigned int voltage;


// read data from file and return it
String readFromFile(const char *filename, String s) {
  s = "";
  if (SPIFFS.exists(filename)) {
    File f = SPIFFS.open(filename,"r");
    if (!f) {
      s = "";
    } else {
      s = f.readString();
      f.close();
    }
  } else {
  }
  // remove trailing cr
  s.trim();
  return s;
}

// write into file
String writeFile(const char *filename, String s) {
  File f = SPIFFS.open(filename, "w");
  f.println(s);
  f.close();
  return s;
}

String htmlTableRow(String s1, String s2) {
  String sRow =
  String("<tr><td>" + s1 + "</td><td>" + s2 + "</td></tr>\n");
  return sRow;
}

String htmlInput(const char* id, String s) {
  String sIn =
  String("<input id=\"" + String(id) + "\" value=\"" + s + "\">");
  return sIn;
}

String htmlSetupPage() {
  String htmlPage =
  String("HTTP/1.1 200 OK\r\n") +
            "Content-Type: text/html\r\n" +
            "Connection: close\r\n" +  // the connection will be closed after completion of the response
            "\r\n" +
            "<!DOCTYPE HTML>" +
            "<html>" +
            "<h1>"+ Smyname + "</h1>" +
            "<hr>" +
            "<form method=\"post\">" +
            "<table>" +
              htmlTableRow(String("Hostname:"),htmlInput(fn_myname,Smyname)) +
              htmlTableRow(String("Site name:"),htmlInput(fn_site,Ssite)) +
              htmlTableRow(String("Location:"), htmlInput(fn_location,Slocation)) +
              htmlTableRow(String("Wifi SSID:"),htmlInput(fn_ssid,Sssid)) +
              // htmlTableRow(String("Password:"),Spass) +
              htmlTableRow(String("MQTT Server:"),htmlInput(fn_mqttserver,Smqttserver)) +
              htmlTableRow(String("<button type=\"submit\">Submit</button>"),String("")) +
            "</table>" +
            "</form>" +
            "</html>" +
            "\r\n";
  return htmlPage;

}

// led funtion. in case of not indoor it does nothing
void setled(byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(0,r,g,b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(n,r,g,b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b, byte show) {
#ifdef NROFLEDS
  led.setPixelColor(n,r,g,b);
  if (show) {
    led.show();
  }
#endif
}

void setled(byte show) {
#ifdef NROFLEDS
  if (!show) {
    int i;
    for (i=0;i<NROFLEDS;i++) {
      setled(i,0,0,0,0);
    }
  }
  led.show();
#endif
}


void printConfig() {
#ifdef DEBUG
  String s;
  Serial.println("Config data in memory / in file:");
  Serial.print(fn_myname);
  Serial.print("=");
  Serial.print(Smyname);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_myname,s));
  
  Serial.print(fn_ssid);
  Serial.print("=");
  Serial.print(Sssid);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_ssid,s));
  
  Serial.print(fn_pass);
  Serial.print("=");
  Serial.print(Spass);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_pass,s));
  
  Serial.print(fn_site);
  Serial.print("=");
  Serial.print(Ssite);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_site,s));
  
  Serial.print(fn_location);
  Serial.print("=");
  Serial.print(Slocation);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_location,s));
  
  Serial.print(fn_mqttserver);
  Serial.print("=");
  Serial.println(Smqttserver);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttserver,s));

#endif
}

void setup() {

#ifdef DEBUG
    Serial.begin(115200);
   // USE_SERIAL.setDebugOutput(true);
    Serial.println("\nStarting");
    Serial.print("Sleep mode:");
    Serial.println(WiFi.getSleepMode());
    Serial.print("Phy mode:");
    Serial.println(WiFi.getPhyMode());
    Serial.print("Reset reason:");
    Serial.println(ESP.getResetReason());
    Serial.print("Reset info:");
    Serial.println(ESP.getResetInfo());
#endif

#ifdef OUTDOOR
  // before doing anything check if we have enough power
  voltage = ESP.getVcc();
  if (voltage <= VOLTAGE_MIN) {
#ifdef DEBUG
    Serial.print("Voltage below minimum: ");
    Serial.println(voltage);
#endif
    ESP.deepSleep(1000000*60*10); // Hibernate 10 minutes.
    delay(100);
  }
#endif 

#ifdef NROFLEDS
  led.begin();
  led.show();
#endif

  setled(255,0,0);
  // setup filesystem
  SPIFFS.begin();

  // read configs from files
  Smyname = readFromFile(fn_myname,Smyname);
  Sssid = readFromFile(fn_ssid,Sssid);
  Spass = readFromFile(fn_pass,Spass);
  Smqttserver = readFromFile(fn_mqttserver,Smqttserver);
  Ssite = readFromFile(fn_site,Ssite);
  Slocation = readFromFile(fn_location,Slocation);

#ifdef DEBUG
  printConfig();
#endif

  WiFi.persistent(false);
  WiFi.hostname(Smyname);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(Sssid.c_str(), Spass.c_str());
  
#ifdef DEBUG
  Serial.print("Connecting to ");
  Serial.println(Sssid);
#endif

  setled(255,128,0);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    retries++;
    #ifdef DEBUG
    Serial.print(".");
    #endif
    if (retries > 100) {
      // no connection after lot of retries
      runMode |= RM_CONFIG;
      // break;
    }
  }

  setled(0,255,0);


  if (WiFi.status() == WL_CONNECTED) {
    runMode &= ~RM_START;
    runMode |= RM_SENSOR;
  }
#ifdef DEBUG
  Serial.println("");
  Serial.println("Wifi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print("/");
  Serial.println(WiFi.subnetMask());
#endif

#ifdef UNDEF
  if (MDNS.begin(Smyname.c_str())) {
    MDNS.addService("http","tcp",80);
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error starting mDNS");
  }
#endif

  // start webserver
  server.begin();

  // setup i2c
  Wire.begin(I2CSDA,I2CSCL);

  // setup light chip
#ifdef LSSENSOR
  ls_setup();
#endif
  delay(10);
  // setup sensor chip
  bme_setup();

#ifdef UVSENSOR
  uv.begin(VEML6070_1_T);
#endif

#ifdef TSL2561
  lightTSL2561.begin();
  #ifdef DEBUG
  unsigned char ID;
  if (lightTSL2561.getID(ID)) {
    Serial.print("Got TSL2561 ID: 0x");
    Serial.print(ID,HEX);
    Serial.println(", should be 0x5x");
  } else {
    byte error = lightTSL2561.getError();
    Serial.print("Got TSL2561 Error: ");
    Serial.println(error);
  }
  #endif
  lightTSL2561.setTiming(gainTSL2561,2,msTSL2561);
  lightTSL2561.setPowerUp();
#endif

  // setup PIR
#ifdef MOTION
  pinMode(pirInput,INPUT);
  pirState = digitalRead(pirInput);
#endif

#ifdef ADC
  ads.begin();
#endif

#ifdef NIKON
  pinMode(NIKONLED,OUTPUT);
#endif

  // setup the mqtt client
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), 1883);
  client.setCallback(callback);

  setled(0,0,0);
}

void callback(char* topic, byte* payload, unsigned int length)  {
  #ifdef DEBUG
  Serial.print("Message arrived[");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
  
  String in;
  for (int i = 0; i < length; i++) {
    in += String((char)payload[i]);
  }
  
  if (in.equals(String("reboot"))) {
    ESP.restart();
  } else if (in.equals(String("sleep"))) {
    sleepFor(60);
  } else if (in.startsWith("led ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int b = in.substring(position).toInt();
    setled(r,g,b);
  } 

  // LEDs off
  else if (in.startsWith("ledsoff")) {
    setled(0);
  }
  
  // LED code in binary
  else if (in.startsWith("ledb ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    byte thisLED = 0;
    position++;
    while (position+2 < length) {
      setled(thisLED, payload[position+0],payload[position+1],payload[position+2],0);
      position+=3;
      thisLED++;
    }
    setled(1);
  }
  else if (in.startsWith("ledn ")) {
    int position=0;

    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int n = in.substring(position).toInt();
    position++;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    int b = in.substring(position).toInt();
    setled(n,r,g,b);

#ifdef NIKON
  } else if (in.startsWith("nikon")) {
    NIKONshoot();
#endif

  // Location  
  } else if (in.startsWith("location ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    writeFile(fn_location,in.substring(position+1));
    printConfig();
  }

  // Mqttserver
  else if (in.startsWith("mqttserver ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    writeFile(fn_mqttserver,in.substring(position+1));
    printConfig();
  }

  // Site
  else if (in.startsWith("site ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    writeFile(fn_site,in.substring(position+1));
    printConfig();
  }

  
  else { 
    #ifdef DEBUG
    Serial.println("unknown command received: " + in);
    #endif
  }
}

void sleepFor(unsigned seconds) {
#ifdef OUTDOOR
  setled(0,0,0);
#ifdef LSSENSOR
  ls_shutdown(); // shutdown light sensor
#endif
  client.disconnect(); //disconnect from MQTT
  delay(100);
  WiFi.disconnect(); // disconnect from Wifi
  delay(200);
  ESP.deepSleep(1000000*seconds);
  delay(100);
#endif
}

boolean reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic,50,"/%s/%s/status",Ssite.c_str(),Smyname.c_str());



#ifdef DEBUG
  Serial.print("Attempting MQTT connection...");
  Serial.print(client.state());
  Serial.print("...");
#endif
  // Attempt to connect
  // if (client.connect(Smyname.c_str(),mytopic,0,0,"stopped")) {
  if (client.connect(Smyname.c_str())) {
#ifdef DEBUG
    Serial.println("connected");
#endif
    // Once connected, publish an announcement...
      
    client.publish(mytopic,"started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
    delay(10);
  } else {
#ifdef DEBUG
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
#endif
  }
  return client.connected();
}

#ifdef LSSENSOR
/******************************************** Lichtsensor */
#define LS_I2C_ADDR     0x29
#define LS_REG_CONTROL  0x00
#define LS_REG_CONFIG   0x01
#define LS_REG_DATALOW  0x04
#define LS_REG_DATAHIGH 0x05
#define LS_REG_ID       0x0A

void ls_setup() {
#ifdef DEBUG
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80|LS_REG_ID);
  Wire.endTransmission();

  Serial.print("LS ID: ");
  Wire.requestFrom(LS_I2C_ADDR, 1); // request 1 byte
  while (Wire.available()) {
    unsigned char c = Wire.read();
    Serial.print( c & 0xF0, HEX);
  }
  Serial.println();
#endif

  delay(100);
  // Power on
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80|LS_REG_CONTROL);
  Wire.write(0x03); //power on
  Wire.endTransmission();
  delay(100);

  // Config
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80|LS_REG_CONFIG);

  Wire.write(LS_FACTOR_M - 1);
  // Wire.write(0x00); //M=1 T=400ms
  // Wire.write(0x01); //M=2 T=200ms
  // Wire.write(0x02); //M=4 T=100ms
  Wire.endTransmission();
}

void ls_shutdown() {
  // Power off
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80|LS_REG_CONTROL);
  Wire.write(0x00); //power on
  Wire.endTransmission();
}

uint32_t ls_read() {
  uint16_t l,h;
  uint32_t lux;

  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80|LS_REG_DATALOW);
  Wire.endTransmission();
  Wire.requestFrom(LS_I2C_ADDR, 2); //request 2 bytes
  l = Wire.read();
  h = Wire.read();
  while(Wire.available()){ Wire.read(); } //received more bytes?
  lux  = (h<<8) | (l<<0);
  lux *= LS_FACTOR_M;
  // lux *= 1; //M=1
  // lux *= 2; //M=2
  // lux *= 4; //M=4
#ifdef DEBUG
  Serial.print("Lux: ");
  Serial.println(lux, DEC);
#endif

  return lux;
}
#endif

/******************************************** Wettersensor */
BME280 wetterSensor;
void bme_setup() {
#ifdef BME280ADDR
  wetterSensor.settings.commInterface = I2C_MODE;
  wetterSensor.settings.I2CAddress = BME280ADDR;
  wetterSensor.settings.runMode = 3;
  wetterSensor.settings.tStandby = 0;
  wetterSensor.settings.filter = 0;
  wetterSensor.settings.tempOverSample = 1;
  wetterSensor.settings.pressOverSample = 1;
  wetterSensor.settings.humidOverSample = 1;

  delay(10);
#ifdef DEBUG
  Serial.print("Starting BME280... result of .begin(): 0x");
  Serial.println(wetterSensor.begin(), HEX);
#else
  wetterSensor.begin();
#endif
#endif
}

#ifdef REGEN_ADC
/************** Regensensor */
int16_t regen() {
  int16_t rain;
  rain = ads.readADC_SingleEnded(REGEN_ADC);
#ifdef DEBUG
  Serial.print("Regen: ");
  Serial.println(rain);
#endif
  return rain;
}
#endif

/**** Bodenfeuchte-Sensor */
#ifdef SOIL_ADC
int16_t soil() {
  int16_t soil;
  soil = ads.readADC_SingleEnded(SOIL_ADC);
#ifdef DEBUG
  Serial.print("Soil: ");
  Serial.println(soil);
#endif
  return soil;
}
#endif


/************* Nikon Infrared Shoot Command */
#ifdef NIKON
void NIKONon(int pin, int time) {
  static const int period = 25;
  // found wait_time by measuring with oscilloscope
  static const int wait_time = 9;

  for (time = time/period; time > 0; time--) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(wait_time);
    digitalWrite(pin, LOW);
    delayMicroseconds(wait_time);
  }
}


void cameraSnap(int pin)
{
// These Timing are from: http://www.bigmike.it/ircontrol/
NIKONon(pin,2000);
//This Delay is broken into 3 lines because the delayMicroseconds() is only accurate to 16383. http://arduino.cc/en/Reference/DelayMicroseconds
delayMicroseconds(7830);
delayMicroseconds(10000);
delayMicroseconds(10000);
NIKONon(pin,390);
delayMicroseconds(1580);
NIKONon(pin,410);
delayMicroseconds(3580);
NIKONon(pin,400);
}

void NIKONshoot() {
  cameraSnap(NIKONLED);
}
#endif

unsigned long lastReconnectAttempt = 0;
void myPublish(char *topic, char *msg) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();
#ifdef DEBUG
  Serial.print("Publish message: ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(msg);
#endif
  client.publish(topic,msg);
}


unsigned int loopDelay = 2000;
int lastMotion = 0, thisMotion = 0;

void loop() {
  
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();
  
  if (runMode & RM_CONFIG) {
    webClient = server.available();
  }

#ifdef UNDEF
  if (webClient) {
    Serial.println("Webclient connected");
    while (webClient.connected()) {
      if (webClient.available()) {
        String line = webClient.readStringUntil('\r');
        Serial.print(line);
        if (line.length() == 1 && line[0] == '\n') {
          webClient.println(htmlSetupPage());
          break;
        }
      }
    }
    delay(1);
    webClient.stop();
    Serial.println("Client disconnected");
  }
#endif
#ifdef MOTION
  thisMotion = digitalRead(pirInput);
#else
  thisMotion = lastMotion;
#endif
  long now = millis();
  if ((now - lastMsg > loopDelay) || (lastMotion != thisMotion)) {
    lastMsg = now;
    ++value;

#ifdef OUTDOOR
    voltage = ESP.getVcc();
    snprintf(topic,50,"/%s/%s/voltage", Ssite.c_str(), Smyname.c_str());
    snprintf(msg,50,"%s",String(voltage / 1000.0,3).c_str());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif

#ifdef LSSENSOR
    snprintf(topic,50,"/%s/%s/light", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%u", ls_read());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif

#ifdef TSL2561
    unsigned int tsldata0,tsldata1;
    if (lightTSL2561.getData(tsldata0,tsldata1)) {
      double lux;
      boolean good;
      good = lightTSL2561.getLux(gainTSL2561,msTSL2561,tsldata0,tsldata1,lux);
      if (good) {
        snprintf(topic,50,"/%s/%s/light", Ssite.c_str(), Slocation.c_str());
        snprintf(msg,50,"%u",(unsigned int) lux);
        if (value > 2) {
          myPublish(topic,msg);
        }
      }
    }
#endif

#ifdef UVSENSOR
    snprintf(topic,50,"/%s/%s/UV/light", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%u", uv.readUV());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif

#ifdef REGEN_ADC
    snprintf(topic,50,"/%s/%s/rain", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%u", regen());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif

#ifdef SOIL_ADC
    snprintf(topic,50,"/%s/%s/soil", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%u", soil());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif


#ifdef BME280ADDR
    snprintf(topic,50,"/%s/%s/temperature", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readTempC(),2).c_str());
    if (value > 2) {
      myPublish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/airpressure", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readFloatPressure()/100,2).c_str());
    if (value > 2) {
      myPublish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/humidity", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readFloatHumidity(),2).c_str());
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif
#ifdef MOTION
    lastMotion = thisMotion;
    snprintf(topic,50,"/%s/%s/motion", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%d", thisMotion);
    if (value > 2) {
      myPublish(topic,msg);
    }
#endif

    if (value > 2) {
      if (voltage >= VOLTAGE_PS) {
         // on power supply
         loopDelay = 60000;
      } else if (voltage <= VOLTAGE_LOW) {
        sleepFor(280);
      } else {
        // on battery, standard voltage
        sleepFor(3*60);
      }
      
    } else {
      
    }
  }
}


