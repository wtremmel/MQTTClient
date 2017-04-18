
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
#include <FS.h>

#include <SparkFunBME280.h>
#include <ESP8266HTTPClient.h>

#define DEBUG 1
#define OUTDOOR 1
// #define INDOOR 1

//Pin defintions
#define I2CSDA 4
#define I2CSCL 5
#define PIRINPUT 12
#define BUZZER 13
#define NEOPIXEL 14

#ifdef INDOOR
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led = Adafruit_NeoPixel(1,NEOPIXEL,NEO_GRB+NEO_KHZ800);
#endif

// Voltage when on power supply
#define VOLTAGE_PS 2800
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
PubSubClient client(espClient);
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
#ifdef INDOOR
  led.setPixelColor(0,r,g,b);
  led.show();
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

#ifdef INDOOR
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
  Serial.println("Config data:");
  Serial.print(fn_myname);
  Serial.print("=");
  Serial.println(Smyname);
  Serial.print(fn_ssid);
  Serial.print("=");
  Serial.println(Sssid);
  Serial.print(fn_pass);
  Serial.print("=");
  Serial.println(Spass);
  Serial.print(fn_site);
  Serial.print("=");
  Serial.println(Ssite);
  Serial.print(fn_location);
  Serial.print("=");
  Serial.println(Slocation);
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

  if (MDNS.begin(Smyname.c_str())) {
    MDNS.addService("http","tcp",80);
#ifdef DEBUG
    Serial.println("mDNS responder started");
  } else {
    Serial.println("Error starting mDNS");
#endif
  }

  // start webserver
  server.begin();

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
  } else if (in.startsWith("location ")) {
    int position=0;
    while (in.substring(position,position+1) != " " && position < in.length()){
      position++;
    }
    Slocation = writeFile(fn_location,in.substring(position+1));
#ifdef DEBUG
    String newLocation;
    newLocation = readFromFile(fn_location,newLocation);
    Serial.print("New Location: \"");
    Serial.print(Slocation);
    Serial.print("=");
    Serial.print(newLocation);
    Serial.println("\"");
#endif
  }
  else { 
    #ifdef DEBUG
    Serial.println("unknown command received: " + in);
    #endif
  }
}

void sleepFor(unsigned seconds) {
  setled(0,0,0);
  ls_shutdown(); // shutdown light sensor
  client.disconnect(); //disconnect from MQTT
  delay(100);
  WiFi.disconnect(); // disconnect from Wifi
  delay(200);
  ESP.deepSleep(1000000*seconds);
  delay(100);
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
  if (client.connect(Smyname.c_str(),mytopic,0,0,"stopped")) {
#ifdef DEBUG
    Serial.println("connected");
#endif
    // Once connected, publish an announcement...
      
    client.publish(mytopic,"started");
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
  } else {
#ifdef DEBUG
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
#endif
  }
  return client.connected();
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



unsigned int loopDelay = 2000;
unsigned long lastReconnectAttempt = 0;
void loop() {

  voltage = ESP.getVcc();
  
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }

  if (runMode & RM_CONFIG) {
    webClient = server.available();
  }

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

  long now = millis();
  if (now - lastMsg > loopDelay) {
    lastMsg = now;
    ++value;
    #ifdef DEBUG
    Serial.print("Publishing: ");
    Serial.println(now);
    #endif

    snprintf(topic,50,"/%s/%s/voltage", Ssite.c_str(), Smyname.c_str());
    snprintf(msg,50,"%s",String(voltage / 1000.0,3).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/light", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%u", ls_read());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/temperature", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readTempC(),2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/airpressure", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readFloatPressure()/100,2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

    snprintf(topic,50,"/%s/%s/humidity", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%s",String(wetterSensor.readFloatHumidity(),2).c_str());
    if (value > 2) {
      client.publish(topic,msg);
    }

#ifdef INDOOR
    snprintf(topic,50,"/%s/%s/motion", Ssite.c_str(), Slocation.c_str());
    snprintf(msg,50,"%d", digitalRead(pirInput));
    if (value > 2) {
      client.publish(topic,msg);
    }
#endif

    if (value > 2) {
      if (voltage >= VOLTAGE_PS) {
         // on power supply
         loopDelay = 3000;
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

