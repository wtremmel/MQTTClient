/**
   MQTTCLient.ino

   Reads light and weather sensors and publishes readings via mqtt

    Created on: 25.3.2017

*/

// Go to longer intervals
#define VOLTAGE_LOW 2040
// Minimum volate for operation
#define VOLTAGE_MIN 1900

//Pin defintions
#define I2CSDA 4  //D2 gruen
#define I2CSCL 5  //D1 gelb
#define PIRINPUT 12 //D6
// Regenzaehler 13
#define NEOPIXEL 14 //D5
#define UNUSED1 2 // D4
#define DUSTTX // D8
#define DUSTRX // D7

// #define ESP1 // Test Arbeitszimmer
// #define ESP2 // Kueche
// #define ESP3 // Wohnzimmer
// #define ESP4 // Garten
// #define ESP6 // Test mit Lithium-Akku
// #define ESP7  // Hausanschlussraum
// #define ESP8 // Fernsehzimmer
// #define ESP9 // Heizraum
// #define ESP10 // Büro Wolfgang / Schifferstadt / Flur 1.OG
// #define ESP11 // Lolin32 Lite
// #define ESP12 // Schlafzimmer
#define ESP13 // Lichterkette / Uhr

#if defined(ESP1)
// #define OUTDOOR 1
#define INDOOR 1
#define NROFLEDS 1
// #define VOLTAGE_PS 
// #define BME280ADDR 0x76
#define BME680ADDR 0x77
#define GY49 0x4a
// #define SI7021 0x40
#define SDS011RX D4
#define SDS011TX D3
#define DEBUG 1
// #define LS_FACTOR_M 0x02
// #define LSSENSOR 1
// #define UVSENSOR 1
// #define RAINCOUNTER 13
// Voltage when on power supply
#define VOLTAGE_PS 3000

#elif defined(ESP2)
#define INDOOR 1
#define MOTION
#define NROFLEDS 10
#define BME280ADDR 0x77
#define LS_FACTOR_M 0x01
// Voltage when on power supply
#define VOLTAGE_PS 2700

#elif defined(ESP3)
#define INDOOR 1
#define BME280ADDR 0x76
#define LSSENSOR
#define LS_FACTOR_M 0x02
#define DEBUG 1
#define NROFLEDS 1
// Voltage when on power supply
#define VOLTAGE_PS 2700

#elif defined(ESP4)
#define DEBUG 1
#define OUTDOOR 1
#define NROFLEDS 1
#define TSL2561
#define BME280ADDR 0x76
#define LS_FACTOR_M 0x02
#define ADC 1
#define REGEN_ADC 0 // ADC-Pin 0 for rain detection
// #define SOIL_ADC 3 // ADC-Pin 3 for soil moisture
// Voltage when on power supply
#define VOLTAGE_PS 2600

#elif defined(ESP6)
#define INDOOR 1
#define VOLTAGE_PS 3300
#define VOLTAGE_LOW 3300
#define DEBUG 1
#define ADXL345 6

#elif defined(ESP7) 
// Indoor with GY49 light sensor
#define INDOOR 1
#define VOLTAGE_PS 3000
#define NROFLEDS 1
#define BME280ADDR 0x76
#define DEBUG 1
#define GY49 0x4a

#elif defined(ESP8) || defined(ESP9) || defined(ESP10) || defined(ESP12)
// Indoor
#define INDOOR 1
#define VOLTAGE_PS 3000
#define NROFLEDS 1
#define BME280ADDR 0x76
#define DEBUG 1
#define TSL2561

#elif defined(ESP11)
#define INDOOR 1
#define VOLTAGE_PS 3000
#define NROFLEDS 1
#define BME280ADDR 0x76
#define DEBUG 1
#define TSL2561

#elif defined(ESP13)
#define INDOOR 1
#define NROFLEDS 60
#define DEBUG 1
#define DS3231 0x68
#define ALEXA
#define CLOCK // behafe like a clock

#endif

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680


#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <Wire.h>
#include <PubSubClient.h>
#include <FS.h>

// to get reset reason
#if defined(ARDUINO_ARCH_ESP32)
#include <rom/rtc.h>
#include <SPIFFS.h>
#endif

#include <SparkFunBME280.h>
// #include <ESP8266HTTPClient.h>

// UV-Sensor - just testing at the moment
#ifdef UVSENSOR
#include "Adafruit_VEML6070.h"
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#endif

// DS3231 real time clock
#if defined(DS3231)
#include "RTClib.h"
RTC_DS3231 rtc;
static byte rtc_initialized = 0;

// Task Scheduler only for Indoor use
#if defined(INDOOR)
#include <TaskScheduler.h>
Scheduler runner;
void ledsshift();
void showTime();
Task ledshift_task(2000,TASK_FOREVER,&ledsshift);
Task clock_task(500,TASK_FOREVER,&showTime);
#endif


// print current time
void printCurrentTime(void) {
  if (rtc_initialized) {
    DateTime now = rtc.now();
    Serial.println("Now: " + String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()));
  } else {
    Serial.println("Error: RTC not initialized");
  }
}

void rtcSetup() {
  if (rtc.begin() && !rtc.lostPower()) {
    rtc_initialized = 1;
#ifdef DEBUG  
    printCurrentTime();
#endif
    } else {
    rtc_initialized = 0;
  }
}
#endif

#if defined(ALEXA)
#include "fauxmoESP.h"
fauxmoESP fauxmo;
#endif


// BME680 Sensor
#if defined(BME680ADDR)
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme680;
#endif

// Temp + Humidity Sensor Si7021
#if defined(SI7021)
#include "Adafruit_Si7021.h"
Adafruit_Si7021 si7021 = Adafruit_Si7021();
#endif

// Dust Sensor SDS011
#if defined(SDS011RX) && defined(SDS011TX)
#include <SDS011.h>
SDS011 sds011;
#endif

// Light sensor TSL2561
#ifdef TSL2561
#include <SparkFunTSL2561.h>
SFE_TSL2561 lightTSL2561;
unsigned int msTSL2561;
boolean gainTSL2561;
#endif

#if defined(ADXL345)
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(ADXL345);
float adxl345Zero = 0;
unsigned int adxlCalibCount = 100;

#define ADXLCALIB 100

float calibrateADXL345 (void) {
  float sum;
  int i;

#ifdef DEBUG
  Serial.print("Calibrating ADXL345. Old value: ");
  Serial.println((unsigned int)(adxl345Zero * 100.0));
#endif

  for (i=0; i < ADXLCALIB; i++) {
    sensors_event_t event; 
    accel.getEvent(&event);
    sum += abs(event.acceleration.x)+abs(event.acceleration.y)+abs(event.acceleration.z);
    delay(50);
  }
  adxl345Zero = sum / ADXLCALIB;
  adxlCalibCount = 100;

#ifdef DEBUG
  Serial.print("Complete. New value:");
  Serial.println((unsigned int)(adxl345Zero * 100.0));
  delay(2000);
#endif
  
  return adxl345Zero;
}

void setup_ADXL345(void) {
  if (!accel.begin()) {
#ifdef DEBUG
    Serial.println("Error detecting ADXL345");
#endif
  }
  
  accel.setRange(ADXL345_RANGE_2_G);

  // get average acceleration
  calibrateADXL345();
}
#else
void setup_ADXL345(void) {}
#endif

#ifdef ADC
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
#endif


#ifdef NROFLEDS
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif
uint32_t snowColor=0;
uint32_t snowInterval=0;

// Run modes
#define RM_START 1
#define RM_SENSOR 2
#define RM_CONFIG 4
#define RM_SNOW 8
uint32_t runMode = RM_START | RM_SENSOR | RM_CONFIG;

#if defined(ARDUINO_ARCH_ESP8266)
ADC_MODE(ADC_VCC);
#endif

const char* fn_ssid = "/ssid";
const char *fn_pass = "/pass";
const char *fn_myname = "/myname";
const char *fn_mqttserver = "/mqttserver";
const char *fn_mqttpass = "/mqttpass";
const char *fn_mqttuser = "/mqttuser";
const char *fn_mqttport = "/mqttport";
const char *fn_site = "/site";
const char *fn_location = "/location";

String Smyname, Spass, Sssid, Smqttserver, Ssite, Slocation, Smqttuser, Smqttpass, Smqttport;

// Web for status and config
// WiFiServer server(80);
WiFiClient espClient, webClient;
PubSubClient client;
// experimental
// WiFiEventHandler disconnectedEventHandler;

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
    File f = SPIFFS.open(filename, "r");
    if (!f) {
#ifdef DEBUG
      Serial.print("Cannot read from file ");
      Serial.println(filename);
#endif
      s = "";
    } else {
      s = f.readString();
      f.close();
    }
  } else {
#ifdef DEBUG
    Serial.print("File ");
    Serial.print(filename);
    Serial.println("does not exist");
#endif
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


uint32_t makeColor(int r, int g, int b) {
#ifdef NROFLEDS
  return led.Color(constrain(r,0,255),constrain(g,0,255),constrain(b,0,255));
#endif
}

uint32_t getColor(int i, uint8_t *r, uint8_t *g, uint8_t *b) {
  uint32_t color = led.getPixelColor(i);
  *r = (uint8_t)(color >> 16);
  *g = (uint8_t)(color >>  8);
  *b = (uint8_t)color;
  return color;
}

// dims all colors equally by factor dimBy
uint32_t dimColor(uint32_t color, int dimBy) {
  uint8_t 
      r = (uint8_t)(color >> 16),
      g = (uint8_t)(color >>  8),
      b = (uint8_t)color;

  // r /= dimBy;
  // g /= dimBy;
  // b /= dimBy;

  // lowest color 
  r = (r == 0) ? 0 : dimBy;
  g = (g == 0) ? 0 : dimBy;
  b = (b == 0) ? 0 : dimBy;
  return makeColor(r,g,b);
}

// round-robin map
uint32_t rrmap(int in) {
#ifdef NROFLEDS
  if (in < 0) {
    return NROFLEDS+in;
  } else if (in >= NROFLEDS) {
    return in-NROFLEDS;
  } else
    return in;
#endif
}

uint32_t addColor(int i, uint32_t color) {
#ifdef NROFLEDS
  if (i < 0 || i >= NROFLEDS) 
    return 0;

  uint8_t r,g,b;
  getColor(i,&r,&g,&b);
  uint8_t 
      rx = (uint8_t)(color >> 16),
      gx = (uint8_t)(color >>  8),
      bx = (uint8_t)color;

  uint32_t result = makeColor(r+rx,g+gx,b+bx);
  led.setPixelColor(i,result);
  return result;
#endif
}

// displays a number of led with the center one brightest
void centerLight(int center, unsigned int width, uint32_t color) {
#ifdef NROFLEDS
  led.setPixelColor(rrmap(center),color);
  for (int i=1; i<=width; i++) {
    led.setPixelColor(rrmap(center+i),dimColor(color,width+1-i));
    led.setPixelColor(rrmap(center-i),dimColor(color,width+1-i));
  }
#endif
}

// additive version - adds Colors together
void centerLight(int center, unsigned int width, uint32_t color, bool additive) {
#ifdef NROFLEDS
  if (!additive)
    return centerLight(center,width,color);

  
  addColor(rrmap(center),color);
  for (int i=1; i<=width; i++) {
    led.setPixelColor(rrmap(center+i),dimColor(color,width+1-i));
    led.setPixelColor(rrmap(center-i),dimColor(color,width+1-i));
  }
#endif
}



void showTime(DateTime when) {
  int hour12;
  if (when.hour() >= 12) {
    hour12 = when.hour()-12;
  } else {
    hour12 = when.hour();
  }

  for (int i=NROFLEDS-1; i>=0; i--) {
    led.setPixelColor(i,0,0,0);
    if (clock_task.isFirstIteration()) {
      led.show();
      delay(10);
    }
  }

  int hourStart = map(hour12,0,12,0,NROFLEDS-1);
  int hourWidth = map(2,0,12,0,NROFLEDS-1) - map(1,0,12,0,NROFLEDS-1);
  float hourFraction= when.minute()/60.0;
  int hourEnd = hourStart + int((float)hourWidth * hourFraction);

  centerLight(hourEnd,3,led.Color(200,0,0));
  centerLight(map(when.minute(),0,59,0,NROFLEDS-1),2,led.Color(200,200,0));
  led.setPixelColor(map(when.second(),0,59,0,NROFLEDS-1),200,200,200);
  led.show();
}

void showTime() {
#if defined(DS3231)
  showTime(rtc.now());
#endif
}

// show the temperature on a scale from minTemp to maxTemp, negative values are blue, positive values are red
void showTemperature(int minTemp, int maxTemp, int thisTemp) {
#ifdef NROFLEDS
#ifdef DEBUG
  Serial.println("showTemperature(" + String(minTemp) + "," + String(maxTemp) + "," + String(thisTemp) + ")");
#endif

  for (int i=0; i < NROFLEDS; i++)
    led.setPixelColor(i,0,0,0);

  if (clock_task.isEnabled()) {
    clock_task.disable();
    clock_task.enableDelayed(5000);
  }
  led.show();

  int lastLed = 0;
  for (int i=minTemp; i<=0 && i <= thisTemp;i++) {
    if (i == thisTemp) {
      led.setPixelColor(map(i,minTemp,maxTemp,0,NROFLEDS-1),0,0,254);
    } else {
      int thisLed = map(i,minTemp,maxTemp,0,NROFLEDS-1);
      led.setPixelColor(thisLed,0,0,1);
      if (thisLed-1 > lastLed)
        led.setPixelColor(thisLed-1,0,0,1);
      lastLed = thisLed;
    }
    led.show();
    delay(10);
  }
  for (int i=1;i<=maxTemp && i <= thisTemp;i++) {
    if (i == thisTemp) {
      led.setPixelColor(map(i,minTemp,maxTemp,0,NROFLEDS-1),254,0,0);
    } else {
      int thisLed = map(i,minTemp,maxTemp,0,NROFLEDS-1);
      led.setPixelColor(thisLed,1,0,0);
      if (thisLed-1 > lastLed)
        led.setPixelColor(thisLed-1,1,0,0);
      lastLed = thisLed;
    }
    led.show();
    delay(10);
  }
  
#endif  
}

// show the humidity on a scale of 0 to 100
void showHumidity(int thisHum) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++)
    led.setPixelColor(i,0,0,0);

  if (clock_task.isEnabled()) {
    clock_task.disable();
    clock_task.restart();
    clock_task.enableDelayed(5000);
  }
  led.show();
  
  for (int i=0; i <= thisHum; i++) {
    if (i == thisHum) {
      led.setPixelColor(map(i,0,100,0,NROFLEDS-1),0,100,100);
    } else if (i < thisHum) {
      led.setPixelColor(map(i,0,100,0,NROFLEDS-1),0,2,2);
    }
    led.show();
    delay(7);
  }
#endif
}

// set all LEDs to the same color
void monochrome(byte r, byte g, byte b) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++) {
    led.setPixelColor(i,r,g,b);
  }
  led.show();
#endif
}

// set LEDs to a variant of the same color (randomized)
void monochrome(byte r, byte g, byte b, byte variant) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++) {
    led.setPixelColor(i,makeColor(r+random(-variant,variant+1),g+random(-variant,variant+1),b+random(-variant,variant+1)));
  }
  led.show();
#endif
}

#if defined(NROFLEDS) && defined(INDOOR)
void ledsshift() {
  uint32_t zeroled;
  zeroled = led.getPixelColor(0);
  for (int i=1; i < NROFLEDS; i++) {
    led.setPixelColor(i-1,led.getPixelColor(i));
  }
  led.setPixelColor(NROFLEDS-1,zeroled);
  led.show();
}

void snow(uint32_t color, int interval) {
    int which = random(0,NROFLEDS);
    int oldcolor = led.getPixelColor(which);
    led.setPixelColor(which, color);
    led.show();
    delay(interval);
    led.setPixelColor(which,oldcolor);
    led.show();
}
#endif

// led funtion. in case of not indoor it does nothing
void setled(byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(0, r, g, b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(n, r, g, b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b, byte show) {
#ifdef NROFLEDS
  led.setPixelColor(n, r, g, b);
  if (show) {
    led.show();
  }
#endif
}

void setled(byte show) {
#ifdef NROFLEDS
  if (!show) {
    int i;
    ledshift_task.disable();
    for (i = 0; i < NROFLEDS; i++) {
      setled(i, 0, 0, 0, 0);
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
  Serial.println(readFromFile(fn_myname, s));

  Serial.print(fn_ssid);
  Serial.print("=");
  Serial.print(Sssid);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_ssid, s));

  Serial.print(fn_pass);
  Serial.print("=");
  Serial.print(Spass);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_pass, s));

  Serial.print(fn_site);
  Serial.print("=");
  Serial.print(Ssite);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_site, s));

  Serial.print(fn_location);
  Serial.print("=");
  Serial.print(Slocation);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_location, s));

  Serial.print(fn_mqttserver);
  Serial.print("=");
  Serial.println(Smqttserver);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttserver, s));

  Serial.print(fn_mqttport);
  Serial.print("=");
  Serial.println(Smqttport);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttport, s));

  Serial.print(fn_mqttuser);
  Serial.print("=");
  Serial.println(Smqttuser);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttuser, s));

Serial.print(fn_mqttpass);
  Serial.print("=");
  Serial.println(Smqttpass);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttpass, s));


#endif
}

#if defined(ARDUINO_ARCH_ESP32)
void verbose_print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}
#endif

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  // USE_SERIAL.setDebugOutput(true);
  Serial.println("\nStarting");
#if defined(ARDUINO_ARCH_ESP8266)
  Serial.print("Sleep mode:");
  Serial.println(WiFi.getSleepMode());
  Serial.print("Phy mode:");
  Serial.println(WiFi.getPhyMode());
  Serial.print("Wifi mode:");
  Serial.println(WiFi.getMode());
  Serial.print("Reset reason:");
  Serial.println(ESP.getResetReason());
  Serial.print("Reset info:");
  Serial.println(ESP.getResetInfo());
#endif
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print("CPU0 reset reason: ");
  verbose_print_reset_reason(rtc_get_reset_reason(0));
  Serial.print("CPU1 reset reason: ");
  verbose_print_reset_reason(rtc_get_reset_reason(1));
  
#endif

#endif

  // Rain counter starts very early
#ifdef RAINCOUNTER
  pinMode(RAINCOUNTER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAINCOUNTER), countRaindrops, FALLING);
#endif

#if defined(ARDUINO_ARCH_ESP8266) && defined(OUTDOOR)
  voltage = ESP.getVcc();

  // before doing anything check if we have enough power
  if (voltage <= VOLTAGE_MIN) {
#ifdef DEBUG
    Serial.print("Voltage below minimum: ");
    Serial.println(voltage);
#endif
    ESP.deepSleep(1000000 * 60 * 10); // Hibernate 10 minutes.
    delay(100);
  }
#endif

#ifdef NROFLEDS
  led.begin();
  led.show();
#endif

  setled(255, 0, 0);
  // setup filesystem
  SPIFFS.begin();

  // read configs from files
  Smyname = readFromFile(fn_myname, Smyname);
  Sssid = readFromFile(fn_ssid, Sssid);
  Spass = readFromFile(fn_pass, Spass);
  Smqttserver = readFromFile(fn_mqttserver, Smqttserver);
  Smqttuser = readFromFile(fn_mqttuser, Smqttuser);
  Smqttpass = readFromFile(fn_mqttpass, Smqttpass);
  Smqttport = readFromFile(fn_mqttport, Smqttport);
  Ssite = readFromFile(fn_site, Ssite);
  Slocation = readFromFile(fn_location, Slocation);

#ifdef DEBUG
  printConfig();
#endif

  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.hostname(Smyname);
#endif
  WiFi.begin(Sssid.c_str(), Spass.c_str());

#ifdef DEBUG
  Serial.print("Connecting to ");
  Serial.println(Sssid);
#endif

  setled(255, 128, 0);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    retries++;
#ifdef DEBUG
    Serial.print("Wifi.status() = ");
    Serial.println(WiFi.status());
    if ((retries % 20) == 0) {
      WiFi.printDiag(Serial);
      Serial.setDebugOutput(true);
      WiFi.waitForConnectResult();
    }
#endif
    if (retries > 100) {
      // no connection after lot of retries
      runMode |= RM_CONFIG;
      // break;
    }
  }

  setled(0, 255, 0);
  delay(100);


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



  // setup i2c
  Wire.begin(I2CSDA, I2CSCL);

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

//Setup real time clock
#if defined(DS3231)
  rtcSetup();
#endif

  // Setup Si7021
#if defined(SI7021)
  si7021.begin();
#endif

#ifdef TSL2561
  lightTSL2561.begin();
#ifdef DEBUG
  unsigned char ID;
  if (lightTSL2561.getID(ID)) {
    Serial.print("Got TSL2561 ID: 0x");
    Serial.print(ID, HEX);
    Serial.println(", should be 0x5x");
  } else {
    byte error = lightTSL2561.getError();
    Serial.print("Got TSL2561 Error: ");
    Serial.println(error);
  }
#endif
  lightTSL2561.setTiming(gainTSL2561, 2, msTSL2561);
  lightTSL2561.setPowerUp();
#endif

#ifdef ADXL345
  setup_ADXL345();
#endif

// GY49 Light Sensor
#ifdef GY49
  Wire.beginTransmission(GY49);
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.endTransmission();
#endif

// BME680 sensor
#if defined(BME680ADDR)
if (!bme680.begin()) {
#ifdef DEBUG
  Serial.println("BME60 not found");
#endif
} else { 
  bme680.setTemperatureOversampling(BME680_OS_8X);
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme680.setGasHeater(320, 150);
}
#endif

// SDS011 Dust Sensor
#if defined(SDS011RX) && defined(SDS011TX)
  Serial.println("Starting SDS011 on Port " + String(SDS011RX) + " and " + String(SDS011TX));
  sds011.begin(SDS011RX,SDS011TX);
#endif

  // setup PIR
#ifdef MOTION
  pinMode(pirInput, INPUT);
  pirState = digitalRead(pirInput);
#endif

#ifdef ADC
  ads.begin();
#endif


  // setup the mqtt client
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), atoi(Smqttport.c_str()));
  client.setCallback(callback);

#if defined(ALEXA)
  fauxmo.enable(true);
  fauxmo.addDevice("Lichterkette");

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state) {
        Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
        if (state) {
          monochrome(0,10,40,30);
        } else {
          setled(0);
        }
  });

  fauxmo.onGetState([](unsigned char device_id, const char * device_name) {
        return 1;
  });

#endif

#if defined(INDOOR)
  // Task Scheduler
  runner.init();
  runner.addTask(ledshift_task);

#endif

  setled(0, 0, 0);
#ifdef CLOCK // start the clock display
  runner.addTask(clock_task);
  clock_task.enable();
#endif
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
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    setled(r, g, b);
  }

  // LEDs off
  else if (in.startsWith("ledsoff")) {
    setled(0);
  }

  // LED code in binary
  else if (in.startsWith("ledb ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    byte thisLED = 0;
    position++;
    while (position + 2 < length) {
      setled(thisLED, payload[position + 0], payload[position + 1], payload[position + 2], 0);
      position += 3;
      thisLED++;
    }
    setled(1);
  }
  else if (in.startsWith("ledn ")) {
    int position = 0;

    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int n = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    setled(n, r, g, b);

  } else if (in.startsWith("monochrome ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int v = in.substring(position).toInt();
    monochrome(r, g, b, v);

  } else if (in.startsWith("ledsshift")) {
    // ledsshift();
    ledshift_task.enable();

  } else if (in.startsWith("snow ")) {  
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int n = in.substring(position).toInt();

    if (n == 0) {
      runMode &= ~RM_SNOW;
    } else {
      runMode |= RM_SNOW;
    }

    snowColor = led.Color(r,g,b);
    snowInterval = n;

    snow(snowColor,n);

    // Location - note that change becomes active after reboot only
  } else if (in.startsWith("location ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_location, in.substring(position + 1));
    printConfig();
  }

  // Mqttserver
  else if (in.startsWith("mqttserver ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttserver, in.substring(position + 1));
    printConfig();
  }

  // Mqttport
  else if (in.startsWith("mqttport ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttport, in.substring(position + 1));
    printConfig();
  }

// Mqttuser
  else if (in.startsWith("mqttuser ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttuser, in.substring(position + 1));
    printConfig();
  }

// Mqttserver
  else if (in.startsWith("mqttpass ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttpass, in.substring(position + 1));
    printConfig();
  }


  // Site
  else if (in.startsWith("site ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_site, in.substring(position + 1));
    printConfig();
  }

  // Unix time if RTC is defined
#if defined(DS3231)
  else if (in.startsWith("unixtime ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t newnow = in.substring(position).toInt();
    printCurrentTime();
    rtc.adjust(DateTime(newnow));
    rtcSetup();
    printCurrentTime();
  }

  else if (in.startsWith("show temperature")) {
    int position = 15;
    
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    if (position == 15) {
      showTemperature(-10,40,(int)getLocalTemperature());
    }
    else
      showTemperature(-10,40,in.substring(position).toInt());

  }

  else if (in.startsWith("show humidity")) {
    int position = 12;
    
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    if (position == 12) {
      showHumidity((int)getLocalHumidity());
    }
    else
      showHumidity(in.substring(position).toInt());

  }

  
  else if (in.startsWith("now")) {
    printCurrentTime();
  }

  else if (in.startsWith("clock")) {
    showTime();
  }
#endif

  else {
#ifdef DEBUG
    Serial.println("unknown command received: " + in);
#endif
  }
}

void sleepFor(unsigned seconds) {
#ifdef OUTDOOR
  setled(0, 0, 0);
#ifdef LSSENSOR
  ls_shutdown(); // shutdown light sensor
#endif


  client.disconnect(); //disconnect from MQTT
  delay(100);
  WiFi.disconnect(); // disconnect from Wifi
  delay(200);
  ESP.deepSleep(1000000 * seconds);
  delay(100);
#endif
}

boolean reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/status", Ssite.c_str(), Smyname.c_str());



#ifdef DEBUG
  Serial.print("Attempting MQTT connection...");
  Serial.print(client.state());
  Serial.print("...");
#endif
  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
  // if (client.connect(Smyname.c_str())) {
#ifdef DEBUG
    Serial.println("connected");
#endif
    // Once connected, publish an announcement...

    client.publish(mytopic, "started");
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
  Wire.write(0x80 | LS_REG_ID);
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
  Wire.write(0x80 | LS_REG_CONTROL);
  Wire.write(0x03); //power on
  Wire.endTransmission();
  delay(100);

  // Config
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_CONFIG);

  Wire.write(LS_FACTOR_M - 1);
  // Wire.write(0x00); //M=1 T=400ms
  // Wire.write(0x01); //M=2 T=200ms
  // Wire.write(0x02); //M=4 T=100ms
  Wire.endTransmission();
}

void ls_shutdown() {
  // Power off
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_CONTROL);
  Wire.write(0x00); //power on
  Wire.endTransmission();
}

uint32_t ls_read() {
  uint16_t l, h;
  uint32_t lux;

  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_DATALOW);
  Wire.endTransmission();
  Wire.requestFrom(LS_I2C_ADDR, 2); //request 2 bytes
  l = Wire.read();
  h = Wire.read();
  while (Wire.available()) {
    Wire.read();  //received more bytes?
  }
  lux  = (h << 8) | (l << 0);
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

/******************** Regenzaehler */
#ifdef RAINCOUNTER
volatile unsigned long int raincounter = 0;
void countRaindrops() {
  ++raincounter;
}
#endif

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
  client.publish(topic, msg);
}

// if any sensor for temperature is defined, return temperature
float getLocalTemperature() {
#if defined(BME280ADDR)
  return wetterSensor.readTempC();
#elif defined(BME680ADDR)
  if (bme680.performReading()) {
      return bme680.temperature;
  } else {
    return 0.0;
  }
#elif defined(SI7021)
  return si7021.readTemperature();
#else
  return 0;
#endif
}

float getLocalHumidity() {
#if defined(BME280ADDR)
  return wetterSensor.readFloatHumidity();
#elif defined(BME680ADDR)
  if (bme680.performReading()) {
      return bme680.humidity;
  } else {
    return 0.0;
  }
#elif defined(SI7021)
  return si7021.readHumidity();
#else
  return 0;
#endif
  
}

unsigned long int loopDelay = 2000;
unsigned long now;
int lastMotion = 0, thisMotion = 0;

void loop() {
#if defined(INDOOR)
  runner.execute();
#endif

  if (!client.connected()) {
    now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();
  fauxmo.handle();

#ifdef MOTION
  thisMotion = digitalRead(pirInput);
#else
  thisMotion = lastMotion;
#endif

#ifdef ADXL345
  sensors_event_t event; 
  accel.getEvent(&event);
  float bump = abs(event.acceleration.x)+abs(event.acceleration.y)+abs(event.acceleration.z);
  float diff = adxl345Zero - bump;
  if (diff > 0.5 || diff < -0.1) {
    snprintf(topic, 50, "/%s/%s/seismo", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%d", (int) (diff *100.0));
    myPublish(topic, msg);
    adxlCalibCount--;
    if (adxlCalibCount == 0) {
      calibrateADXL345();
    }
  }


#endif

  now = millis();
  if ((now - lastMsg > loopDelay) || (lastMotion != thisMotion)) {
    lastMsg = now;
    ++value;

#if defined(OUTDOOR) && defined(ARDUINO_ARCH_ESP8266)
    voltage = ESP.getVcc();
    snprintf(topic, 50, "/%s/%s/voltage", Ssite.c_str(), Smyname.c_str());
    snprintf(msg, 50, "%s", String(voltage / 1000.0, 3).c_str());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef LSSENSOR
    snprintf(topic, 50, "/%s/%s/light", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%u", ls_read());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef TSL2561
    unsigned int tsldata0, tsldata1;
    if (lightTSL2561.getData(tsldata0, tsldata1)) {
      double lux;
      boolean good;
      good = lightTSL2561.getLux(gainTSL2561, msTSL2561, tsldata0, tsldata1, lux);
      if (good) {
        snprintf(topic, 50, "/%s/%s/light", Ssite.c_str(), Slocation.c_str());
        snprintf(msg, 50, "%u", (unsigned int) lux);
        if (value > 2) {
          myPublish(topic, msg);
        }
      }
    }
#endif

#ifdef GY49
    unsigned int gydata[2];
    Wire.beginTransmission(GY49);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(GY49,2);
    if (Wire.available() == 2) {
      gydata[0] = Wire.read();
      gydata[1] = Wire.read();
      int exponent =  (gydata[0] & 0xf0) >> 4;
      int mantissa = ((gydata[0] & 0x0f) << 4) | (gydata[1] & 0x0f);
      float luminance = (float)pow(2,exponent) * (float)mantissa * 0.045;
    
      snprintf(topic, 50, "/%s/%s/light", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%s", String(luminance, 2).c_str());
      if (value > 2) { 
        myPublish(topic, msg);
      }
    } else {
#ifdef DEBUG
      Serial.println("Error: GY49 no data");
#endif
    }
#endif

#ifdef UVSENSOR
    snprintf(topic, 50, "/%s/%s/UV/light", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%u", uv.readUV());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef REGEN_ADC
    snprintf(topic, 50, "/%s/%s/rain", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%u", regen());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef RAINCOUNTER
    snprintf(topic, 50, "/%s/%s/raindrops", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%u", raincounter);
    if (value > 2) {
      myPublish(topic, msg);
      raincounter = 0;
    }
#endif


#ifdef SOIL_ADC
    snprintf(topic, 50, "/%s/%s/soil", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%u", soil());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#if defined(BME280ADDR) || defined(BME680ADDR) || defined(SI7021)
    snprintf(topic, 50, "/%s/%s/temperature", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(getLocalTemperature(), 2).c_str()); 
    if (value > 2) {
      myPublish(topic, msg);
    }

    snprintf(topic, 50, "/%s/%s/humidity", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(getLocalHumidity, 2).c_str());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef BME280ADDR
    snprintf(topic, 50, "/%s/%s/airpressure", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(wetterSensor.readFloatPressure() / 100, 2).c_str());
    if (value > 2) {
      myPublish(topic, msg);
    }

#endif

#if defined(BME680ADDR)
    if (bme680.performReading()) {
      snprintf(topic, 50, "/%s/%s/airpressure", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%s", String(bme680.pressure / 100.0, 2).c_str());
      if (value > 2) {
        myPublish(topic, msg);
      }
    
      snprintf(topic, 50, "/%s/%s/airquality", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%s", String(bme680.gas_resistance / 1000.0, 2).c_str());
      if (value > 2) {
        myPublish(topic, msg);
      }
    } 
#ifdef DEBUG
    else {
      Serial.println("Error getting reading from BME680");
    }
#endif
#endif

#if defined(SDS011RX) && defined(SDS011TX)
    int sds011error;
    float p10,p25;
    // sds011.wakeup();
    delay(100);
    sds011error = sds011.read(&p25,&p10);
    if (!sds011error) {
      snprintf(topic, 50, "/%s/%s/P25", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%s", String(p25, 2).c_str());
      if (value > 2) {
        myPublish(topic, msg);
        // sds011.sleep();
      }
      snprintf(topic, 50, "/%s/%s/P10", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%s", String(p10, 2).c_str());
      if (value > 2) {
        myPublish(topic, msg);
      }
    }
#endif

#ifdef MOTION
    lastMotion = thisMotion;
    snprintf(topic, 50, "/%s/%s/motion", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%d", thisMotion);
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

    if (value > 2) {
#if defined(OUTDOOR) && defined(ARDUINO_ARCH_ESP8266)
      if (voltage >= VOLTAGE_PS) {
        // on power supply
        loopDelay = 30*1000;
      } else if (voltage <= VOLTAGE_LOW) {
        sleepFor(8 * 60);
      } else {
        // on battery, standard voltage
        sleepFor(4 * 60);
      }
#endif
#ifdef INDOOR
      loopDelay = 60*1000;
#endif

    } else {

    }
  }
}



