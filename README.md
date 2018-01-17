# MQTTClient
ESP8266 MQTT Client for sensor reading

## Parts List Indoor Version
* ESP8266 Microcontroller incl. USB port for easier programming
* BME280 Temperature/Pressure/Humidity Sensor
* TSL2561 Light sensor
* WS2812 RGP LEDs (as many as you need)
* PIR motion sensor (not used in Example)
* See parts photo ![Parts](pictures/wt-mqttclient-13.jpg "Parts")

## Parts List Outdoor Version
* ESP8266 Microcontroller incl. USB port for easier programming
* BME280 Temperature/Pressure/Humidity Sensor
* TSL45315 Light sensor
* Solar panel 3-5W (the lower the voltage the better)
* Battery holder for two AA cells
* Two AA Eneloop cells (enough for three cloudy days)
* 3.3V step down converter (depending on solar panel used)
* 3.3V voltage regulator (to prevent battery discharge via the regulator)

# Soldering
PIR is not used in this example. Because I made a mistake with the PCB board, you must solder a wire from the D1mini GND pin to the GND pin of the sensors.

1. First solder the pin headers ![Pin Headers](pictures/wt-mqttclient-14.jpg)
2. Next plug in the D1mini including the male pin headers and solder the male pin headers to the D1mini from the top ![D1mini](pictures/wt-mqttclient-12.jpg)
3. Do the same with the two sensors ![Sensors](pictures/wt-mqttclient-15.jpg)
4. Depending on what type of LED you have solder the LED ![LED](pictures/wt-mqttclient-16.jpg)
5. Finished module ![Complete](pictures/wt-mqttclient-17.jpg)

