/* 
VERSION 3 - SENSOR APP FOR MQTT COMMUNICATION WITH OPENHAB ON RASPBERRY Pi

EXTENSIVE COMMENTING AND TIDY UP OF WHOLE PROGRAM TO MAKE IT MORE READABLE AND EASIER TO EDIT IN FUTURE
*/

//IMPORT SOME LIBRARIES
#include "Arduino.h"
#include <ESP8266WiFi.h>      // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED
#include <PubSubClient.h>
#include <Wire.h>             // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED

//#include "Adafruit_HTU21DF.h" // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED
//#include <Adafruit_BMP085.h>  // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>    // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED - THIS IS THE BME280 AS SUED ON MY DIY SENSOR UNITS
#include <Adafruit_BME280.h>    // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED - THIS IS THE BME280 AS SUED ON MY DIY SENSOR UNITS
#include <Adafruit_AHTX0.h>     // THIS LIBRARY MAY NEED CHANGING DEPENDING ON SENSORS BEING USED - THIS IS THE AHT10 AS SUED ON MY DIY SENSOR UNITS

ADC_MODE(ADC_VCC) // setup availability to read votage from ADC pin

// DEFINE THE CONSTANTS
// SOME CONSTANTS THAT MAY CHANGE FROM SENSOR TO SENSOR
const int MEASUREMENT_PERIOD = 1000000*1800; // TIME PERIOD FOR MEASUREMENTS (1sec * PERIOD) 3600 is 1 hour

//WIFI BROKER CONSTANTS - NETWORK SSID, PASSWORD, MQTT SERVER IP ADDRESS
const char* NETWORK_SSID = "chameleon";       // WIFI SSID
const char* NETWORK_PASSWORD = "colourchange";// WIFI PASSWORD
const char* MQTT_SERVER = "192.168.0.99";     // MQTT BROKER IP ADDRESS
const char* MQTT_PORT = "1883";               // MQTT PORT NUMBER
const char* MQTT_CLIENT = "Hall";         // THIS NAME NEEDS TO BE UNIQUE FOR EACH CLIENT CREATED

const char* SUBSCRIBE_TOPIC_1 = ""; // TOPIC TO SUBSCRIBE TO, ADD MORE FOR EACH NEW TOPIC
const char* PUBLISH_TOPIC_1 = "hall/sensor1";               // TOPIC TO PUBLISH TO


// OTHERS
const int ONBOARD_LED_PIN = 13; // just for testing the onboard LED during commands sent to sensor board
const int SENSOR_PWR_PIN = 15; // powers the sensor chip (GPIO15 - D8)


//Adafruit_HTU21DF HTU_SENSOR = Adafruit_HTU21DF();  //CREATE AND ADAFRUIT HTU21DF OBJECT, THERE ARE NO PINS TO DEFINE AS THE I2C BUS IS 
//Adafruit_BME280 BMP_SENSOR;                        //CREATE AND ADAFRUIT BMP280 OBJECT,THERE ARE NO PINS TO DEFINE AS THE I2C BUS IS USED 
Adafruit_AHTX0 aht;                                 //CREATE AND ADAFRUIT AHT10 OBJECT,THERE ARE NO PINS TO DEFINE AS THE I2C BUS IS USED

//MQTT CONFIGURATION
WiFiClient ESP_CLIENT;                            //CREATES A PARTIALLY INITIALISED CLIENT INSTANCE
PubSubClient client(ESP_CLIENT);                  //CONFIGURE THE SERVER BEFORE IT IS USED


//
void setup() {
  
  Serial.begin(115200);   //ALLOW SERIAL DATA COMMUNICATION, MAINLY FOR ECHOING STUFF TO THE SERIAL MONITOR IN ARDUINO IDE, USEFULL FOR DEBUGGING PROGRAM DURING TESTING STAGE

  // SET THE GPIO PIN TO AN OUTPUT 
 pinMode(ONBOARD_LED_PIN, OUTPUT);
 pinMode(SENSOR_PWR_PIN, OUTPUT);
 

 digitalWrite(SENSOR_PWR_PIN, HIGH);

// CONNECT TO THE WIFI NETWORK WITH DEFINED NETWORK SSID AND PASSWORD
  
  Serial.println();                               //CREATE A COUPLE OF BLANK LINES ON THE SERIAL MONITOR
  Serial.println();
  
  Serial.print("Connecting to NETWORK [");        //MESSAGE ABOUT WHICH NETWORK WE'RE CONECTING TO
  Serial.print(NETWORK_SSID);
  Serial.println("]");

  // CONNECT TO NETWORK AS DEFINED BY NETWORK_SSID AND NETWORK_PASSWORD
  WiFi.mode(WIFI_STA);
  WiFi.begin(NETWORK_SSID, NETWORK_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  // DISPLAY INFO IN THE SERIAL MONITOR ABOUT THE NETWORK CONNECTION
  Serial.println("");
  Serial.println("WiFi connected"); // SAY IF WE'RE CONNECTED TO THE NETWORK 
  Serial.print("Connected to:[");
  Serial.print(NETWORK_SSID);
  Serial.println("]");
  
  Serial.print("IP address:[");   // SAY WHAT THE IP ADDRESS IS
  Serial.print(WiFi.localIP());
  Serial.print("] with connection strength: [");
  Serial.print(WiFi.RSSI());
  Serial.println("]");
  
  // CALL  THE FUNCTION TO CONNECT TO THE MQTT SERVER
  delay(1000);
  connect_to_MQTT();

  setupAHTSensor();
  
 



  // IF THE MQTT CLIENT IS NOT CONNECTED TO THE MQTT SERVER THEN REPORT THIS IN THE SERIAL MONITOR AND TRY TO RECONNECT
  if (! client.connected()) {
    
    Serial.print("[");           // MESSAGE ABOUT OUR CONNECTION THE THE MQTT SERVER
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Not connected to MQTT Server..... Trying to reconnect..");
    connect_to_MQTT();  // TRY TO RECONNECT BY CALLING THE connect_to_MQTT FUNCTION
  }

 
   //NOW = millis(); // ASSIGN CURRENT TIME TO 'NOW'
   
  //PUBLISH DATA AT INTERVALS DEFINED BY MEASUREMENT_PERIOD
  //if (NOW - LAST_MEASURE > MEASUREMENT_PERIOD) {
   // LAST_MEASURE = NOW;
    
    createJSON(); // CALL createJSON FUNCTION TO PARSE DATA INTO JSON FORMAT
  //}

  client.loop();


  Serial.println("I'm awake, but I'm going into deep sleep mode for 15 mins");
  digitalWrite(SENSOR_PWR_PIN, LOW);
  ESP.deepSleep(MEASUREMENT_PERIOD);
  
}


//******************************
// FUNCTIONS TO DO STUFF - CONSIDER MOVING THESE TO AN EXTERNAL FILE
//*****************************

//function to setup AHT10 sensor

void setupAHTSensor(){
   // SETUP THE AHT10 SENSOR
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
}


//function to create the JSon file
void createJSON(){

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
 
  JSONencoder["hall"] = "sensor1";
  JsonArray& temps = JSONencoder.createNestedArray("temperature");
      temps.add(temp.temperature);
    
  JsonArray& hums = JSONencoder.createNestedArray("humidity");
       hums.add(humidity.relative_humidity);
    
  JsonArray& rssi = JSONencoder.createNestedArray("rssi");
       rssi.add(getConnectionData());

  JsonArray& volts = JSONencoder.createNestedArray("voltage");
       volts.add(getVoltage());

  char JSONmessageBuffer[300];
  JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
 
  client.publish(PUBLISH_TOPIC_1,JSONmessageBuffer);
 
  Serial.println(JSONmessageBuffer);
}


// THIS FUNCTION IS USED TO SEND MESSAGES OUT THE THE MQTT CLIENT - CURRENTLY BEING USED TO SEND SWITCH COMMANDS
void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

    for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
    
  }
  Serial.println(messageTemp);
  
  
  if(messageTemp == "ON"){
        digitalWrite(ONBOARD_LED_PIN, HIGH);
       
      }
  else if(messageTemp == "OFF"){
        digitalWrite(ONBOARD_LED_PIN, LOW);
       
      }
 
}


// THIS FUNCTION DEALS WITH GETTING DETAILS ABOUT THE WIFI CONNECTION STRENGTH
float getConnectionData(){
  
  float rssi = (WiFi.RSSI());
  return rssi; // SEND RESULT BACK TO CALLING FUNCTION
  
}

float getVoltage(){

 
  // read the input on analog pin 0:
 
  float voltage = (ESP.getVcc()/1000.00);
    // print out the value you read:
  Serial.println(voltage);
  return voltage;
  }


// THIS FUNCTION DEALS WITH CONNECTING TO THE MQTT SERVER AND SUBSCRIBING TO ANY SUBSCRIBE TOPICS DEFINED BY THE MQTT SERVER APPLICATION EG OPENHAB
void connect_to_MQTT() {

  client.setServer(MQTT_SERVER, 1883);  //SET THE MQTT SERVER DETAILS - DEFINED BY THE MQTT_SERVER VARIABLE AND THE MQTT_PORT VARIABLE (1883)
  client.setCallback(callback);

  if (client.connect(MQTT_CLIENT)) {

    Serial.print("[");           // MESSAGE ABOUT OUR CONNECTION THE THE MQTT SERVER
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Connected to MQTT Server");
    
    //client.subscribe(SUBSCRIBE_TOPIC_1);  // THE TOPICS WE ARE SUBSCRIBING TO - MORE CAN BE ADDED AS REQUIRED
    
     }
     else {

    Serial.print("[");           // MESSAGE ABOUT THE LACK OF ONNECTION THE THE MQTT SERVER
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Could not connect to MQTT Server");
    
  }


}




 

// LOOP ROUND THIS FUNCTION REPEATEDLY 
void loop() {
  
 
 
}
