/*
 * Connnect Wemos as follows
 *   Connect D0 and RST
 * 
 * Connnect SCD30 to Wemos as follows
 *   VIN to 5V
 *   GND to GND
 *   TX/SCL to D1
 *   RX/SDA to D2
 * 
 * !! AFTER CONNECTIONG TO POWER, PRESS RST BUTTON to start the program !!
 */

#include <Arduino.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include "paulvha_SCD30.h"

// WiFi Definitions
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_WIFI_PSSWD";

// Change to the pressure in mbar on your location 
// for better SCD30 - CO2 results (between 700 and 1200 mbar)
#define pressure 990

// Change interval of measurement (in s)
// Default is 2.
#define measureInterval 10

#define temperatureOffset 1.6

// Set SCD30 driver debug level (ONLY NEEDED CASE OF SCD30 ERRORS) 
// 0 : no messages                                                      
// 1 : request sending and receiving                                    
// 2 : request sending and receiving + show protocol errors             
#define scd_debug 0

// Set Loxone IP and Port
IPAddress sendIpAddress(10, 200, 1, 10);
unsigned int sendIpPort = 7888;

//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

#define SCD30WIRE Wire

SCD30 airSensor;

float temp;
float hum;
float co2;
int detect_SCD30 = 0;

WiFiUDP udpSender;

char scd30SerialNumber[(SCD30_SERIAL_NUM_WORDS * 2) +1];

void setupWiFi() {
  // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println(F("WiFi connected"));
}

void initHardware() {
  Serial.begin(115200);

  // disable Wemos build-in blue led
  // does not work after start (after wakeup), so blue led blink anyway
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);

  // deep-sleep: for automatic wakeup
  pinMode(D0, WAKEUP_PULLUP);

  // setup I2C
  SCD30WIRE.begin();

  // set SCD30 driver debug level (only in case of errors)
  // requires serial monitor (remove DTR-jumper before starting monitor)
  airSensor.setDebug(scd_debug);

  // This will cause SCD30 readings to occur every two seconds
  // will init BUT not start reading
  if (! airSensor.begin(SCD30WIRE, false)) {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }
  
  // Start Auto self calibration
  // airSensor.setAutoSelfCalibration(1);

  // Set Force CO2concentration value, outside is 414
  //airSensor.setForceRecalibration(420);

  // Pressure adjustment
  airSensor.setAmbientPressure(pressure); //Current ambient pressure in mBar: 700 to 1200
  
  // Temperature offset
  airSensor.setTemperatureOffset((uint16_t)0); // set to 0, because other values does not work correctly

  // Time interval
  airSensor.setMeasurementInterval(measureInterval);

  detect_SCD30 = 1;

  // Read SCD30 serial number as printed on the device
  delay(1000);
  airSensor.getSerialNumber(scd30SerialNumber);
}

void setup() {
  initHardware();
  setupWiFi();

  delay(1000);
}

void loop() {
  if (detect_SCD30 != 1) {
    Serial.println("SCD30 not detected");
    return;
  }

  // Prepare the message. 
  Serial.println("Preparing message ...");
  String s = "co2 ";
  s += scd30SerialNumber;

  temp = airSensor.getTemperature();
  hum = airSensor.getHumidity();
  co2 = airSensor.getCO2();

  if (co2 < 414) {
      co2 = 414;
  }

  if (co2 < 2500 &&
      temp > 0 && temp < 35 &&
      hum >5 && hum < 80) {
        s += " state_ok ";
  } else {
        s += " state_error ";
  }

  s += (temp - temperatureOffset);
  s += " ";

  s += hum;
  s += " ";

  s += co2;

  // Send return packet
  Serial.println("Sending message: " + s);
  udpSender.beginPacket(sendIpAddress, sendIpPort);
  udpSender.print(s);
  udpSender.endPacket();
  Serial.println("Message has been send");

  delay(1000);
  
  // deep sleep
  Serial.printf("I'm awake, but I'm going into deep sleep mode for %d seconds", 60);
  ESP.deepSleep(60e6, WAKE_RF_DEFAULT);
}

