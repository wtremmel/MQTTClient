/**
 * MQTTCLient.ino
 * 
 * Reads light and weather sensors and publishes readings via mqtt
 *
 *  Created on: 25.3.2017
 *
 */

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <SparkFunBME280.h>

#include <ESP8266HTTPClient.h>

#define DEBUG 1
// #define MOTION 1

// Voltage when on power supply
# define PSVOLTAGE 28000

// Run modes
#define RM_SENSOR 1
#define RM_CONFIG 2

//Pin defintions
#define I2CSDA 4
#define I2CSCL 5
#define PIRINPUT 12
#define BUZZER 13

ADC_MODE(ADC_VCC);

// These should be configurable at runtime
const char* ssid = "AnkhMorpork";
const char* pass = "TheC0l0r0fMag1c";
const char* myname = "esp1";
const char* mqttserver = "pi3.garf.de";
const char* site = "Chattenweg5";
const char* location = "Arbeitszimmer";


WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
char topic[50];
int value = 0;

int pirInput = PIRINPUT;
int pirState = LOW;

unsigned int voltage;


void setup() {

#ifdef DEBUG
    Serial.begin(115200);
   // USE_SERIAL.setDebugOutput(true);

    Serial.println("Starting");

#endif

  WiFi.mode(WIFI_STA);
  WiFi.hostname(myname);
  WiFi.begin(ssid, pass);

#ifdef DEBUG
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG
    Serial.print(".");
    #endif
  }

#ifdef DEBUG
  Serial.println("");
  Serial.println("Wifi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print("/");
  Serial.println(WiFi.subnetMask());
#endif

  if (MDNS.begin(myname)) {
    MDNS.addService("http","tcp",80);
#ifdef DEBUG
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error starting mDNS");
#endif
  }

  // setup i2c
  Wire.begin(I2CSDA,I2CSCL);

  // setup light chip
  ls_setup();
  delay(10);
  // setup sensor chip
  bme_setup();

  // setup PIR
#ifdef MOTION
  pinMode(pirInput,INPUT);
  pirState = digitalRead(pirInput);
#endif

  // setup the mqtt client
  client.setServer(mqttserver, 1883);
  client.setCallback(callback);

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
    ESP.reset();
  } else if (in.equals(String("sleep"))) {
    sleepFor(60);
  }
  else { 
    #ifdef DEBUG
    Serial.println("unknown command received: " + in);
    #endif
  }
}

void sleepFor(unsigned seconds) {
  ls_shutdown(); // shutdown light sensor
  client.disconnect(); //disconnect from MQTT
  WiFi.disconnect(); // disconnect from Wifi
  
  ESP.deepSleep(1000000*seconds);
  delay(100);
}

void reconnect() {
  // Loop until we're reconnected
  snprintf(topic,50,"/%s/%s/status",site,myname);

  
  while (!client.connected()) {
#ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if (client.connect(myname,topic,0,0,"stopped")) {
#ifdef DEBUG
      Serial.println("connected");
#endif
      // Once connected, publish an announcement...
      
      client.publish(topic,"started");
      // ... and resubscribe to my name
      client.subscribe(myname);
    } else {
#ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
#endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

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
  Wire.write(0x00); //M=1 T=400ms
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
  lux *= 1; //M=1
  // lux *= 2; //M=2
  // lux *= 4; //M=4
#ifdef DEBUG
  Serial.print("Lux: ");
  Serial.println(lux, DEC);
#endif

  return lux;
}

/******************************************** Wettersensor */
BME280 wetterSensor;
void bme_setup() {
  wetterSensor.settings.commInterface = I2C_MODE;
  wetterSensor.settings.I2CAddress = 0x77;
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
}




void loop() {

  voltage = ESP.getVcc();
  
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    #ifdef DEBUG
    snprintf(topic,50,"/%s/%s/test", site, location);
    snprintf(msg,50,"%ld", value);
    Serial.print("Publishing: ");
    Serial.println(msg);
    // client.publish(topic,msg);
    #endif

    snprintf(topic,50,"/%s/%s/voltage", site, myname);
    snprintf(msg,50,"%s",String(voltage / 1000.0,2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/light", site, location);
    snprintf(msg,50,"%u", ls_read());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/temperature", site, location);
    snprintf(msg,50,"%s",String(wetterSensor.readTempC(),2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/airpressure", site, location);
    snprintf(msg,50,"%s",String(wetterSensor.readFloatPressure()/100,2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/humidity", site, location);
    snprintf(msg,50,"%s",String(wetterSensor.readFloatHumidity(),2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

#ifdef MOTION
    snprintf(topic,50,"/%s/%s/motion", site, location);
    snprintf(msg,50,"%d", digitalRead(pirInput));
    if (value > 2) {
      client.publish(topic,msg);
    }
#endif

    if (value > 2) {
      if (voltage < PSVOLTAGE) {
        // on battery
        sleepFor(3*60);
      } else {
        // on power supply
        delay(1000*60);
      }
    }
  }

}

