/**********************************************************************
  Filename    : Gartenbewaesserung_20200921 - untested version
  Project     : Gartenbewaesserung Roemerweg
  Author      : JPJ
  Last Modification: 2020/09/21
  Target: Control of two water pipelines. Water can be send to either one or both water lines. 
          Water flow rate is determind and used for S- and S+ control as well as to provide data for calculating water consumption (daily, weekly, monthly, yearly).
          Send all values (status and flow rates, summed volumes) to FHEM-OS by MQTT.
          Accepting commands via MQTT for setting valves.  
  Description of equipment and wiring:
          - 1 Hall flow meter for water flow measurement
          - 3 Magnet-Valves for control of flow (direction) - powered by one 12V Trafo
          - 3 5V Relais for contolling the 12V Magnet Valves - powered by one 12V Trafo
          - BME280 for measuring temperature, pressure, humidity
          - WEMOS D1 Mini for program execution, MQTT communication, sensor input and contolling relais - powered by WEMOS D1 MIni 5V out 
  Wiring of equipment:
          - 5 V to USB of Wemos-Battery shield
          - 12 V to actor-side of relais (12V ground to magnetic valves)
          - Relais N1,N2,N3,N4 input to: Wemos D3, D6, D4, D8  (GPIO: 0, 12, 2, 15)
          - BME280: VIN: 3.3V; GND: ground;SCL: D1(GPIO: 5);SDA: D2(GPIO: 4)
          - Flow Sensor: D7(GPIO: 13), VIN: 3.3V; GND: ground
          - LED:
            - green: on when relai is on - wired in parellel to the magnetic valves
            - red: on when Wemos is online - short blink during MQTT data transfer    --- not implemented yet

BME280: I2C device found at address 0x76  !
==>has to be changed in the Adafruit_BME280.h libary to this adress!!!
**********************************************************************/

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureAxTLS.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureAxTLS.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <dummy.h>
#include <math.h>
#include <PubSubClient.h>

#include "config.h"

Adafruit_BME280 bme; // I2C

//WLAN infos
const char* host = WIFI_host;

// WiFi settings
const char* ssid = WIFI_ssid;
const char* password = WIFI_password;

WiFiClient net;
PubSubClient client(net);

int Relay1 = 15;// D8
int Relay1a = 0;// D3
int Relay1b = 2;// D4
//int Relay2 = 14;// D5
int buttonPin = 13; // D7

int tellstate = 0;

float h, t, p, pin, dp;
char temperatureString[6];
char dpString[6];
char humidityString[6];
char pressureString[7];
char* Relay1StateString;
char* Relay1aStateString;
char* Relay1bStateString;
char* Relay2StateString;
char* AlarmText;
char* timeWarningText;

float calibrationFactor = 2.5;  // 2.5 pulses/second === 1 L/min   <====== this must be calibrated when setup was completed
volatile byte pulseCount;
volatile byte countedPulses;
unsigned long oldTime;
unsigned long timeDifference;
float flowRateLmin;
float flowRateLh;
float flowRate;
unsigned long totalLitres;
unsigned long totalMilliLitres;
unsigned long totalMicroLitres;

char* irrigationTimer = "false";
char* initialTimerRun="true";
long timerTimeLeft;
unsigned long irrigationTime;
unsigned long irrigationTimeMinutes;
unsigned long timerStart;

char* timeWarning="stopp";
unsigned long timeWarningStartTime;


// runs only on boot
void setup() {
  
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("Booting...");

  Wire.begin(D2,D1);
  Wire.setClock(100000);

  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Starting the MQTT
  connect();
  Serial.println("Connection completed...");  
  
  // Printing the ESP IP address
  Serial.println(WiFi.localIP());

  //initialize pins connected to relais
  pinMode(Relay1, OUTPUT);
  digitalWrite(Relay1, HIGH);
  pinMode(Relay1a, OUTPUT);
  digitalWrite(Relay1a, HIGH);
  pinMode(Relay1b, OUTPUT);
  digitalWrite(Relay1b, HIGH);
//  pinMode(Relay2, OUTPUT);
//  digitalWrite(Relay2, HIGH);
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
  Serial.println("PINS defined"); 
  attachInterrupt(digitalPinToInterrupt(buttonPin), pulseCounter, RISING);
  Serial.println("Interrupt defined"); 
  pulseCount = 0;
  flowRate = 0.0;
  totalMilliLitres = 0;
  oldTime = 0;
  Serial.println("Variables defined");
//      delay(4000);   
  startbme();
  Serial.println("End of Setup"); 
}

void loop() {

  //Check and pot. reconnect to WLAN and MQTT
  if (!client.connected()) {
    Serial.println("Not connected to MQTT....");
    connect();
  }
  client.loop();
  getFlowMeterData();                             //Get Flow-Sensor input
  getValveStatus();                              //Get Valve status  

  if ( (millis() - tellstate) > 20000 ) {
    
    getWeather();                                 //Get BME280-Sensor input
    irrigationTimeWarning();   
    mqttPublish();
    tellstate = millis();  
//    digitalWrite(Relay2, LOW);
//    delay(2000);
//    digitalWrite(Relay2, HIGH);
  }                                              //Send Sensor input and Valve status to MQTT-server every 20 seconds
//  safety();
  timer();

}



//
//--------------------------------------------------------------------------------------------------------------
//side functions - begin ---------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//


void closeAllValves(){
  digitalWrite(Relay1, HIGH);    // turn the RELAY off
  digitalWrite(Relay1a, HIGH);    // turn the RELAY off
  digitalWrite(Relay1b, HIGH);    // turn the RELAY off
  Relay1StateString = "relai1off";  
  Relay1aStateString = "relai1aoff";
  Relay1bStateString = "relai1boff";
  timeWarning = "stopp";
}

void irrigationTimeWarning(){
  if(timeWarning == "start"){
    timeWarningStartTime = millis();
    timeWarning = "active";
//    Serial.println("Bewässerungsüberwachung gestartet");
  }
  if(timeWarning == "active"){
    if ((millis() - timeWarningStartTime) > 1800000){      //30 Minuten (1800000 millisekunden)
      timeWarningText = "Bewässerung läuft seit 30 Minuten";
      client.publish("Garten/Bew/timeWarningText", String(timeWarningText).c_str());   // Alarmmeldung alle x Minuten bei aktiver Bewässerung
//      Serial.println("Bewässerung läuft seit 30 Minuten");
      timeWarningStartTime = millis();
    }
  }
}

void timer(){

  if(irrigationTimer == "true"){
    if(initialTimerRun == "true"){
     timerStart = millis();
     initialTimerRun = "false";
     Serial.println("Timer aktiviert für:");
     Serial.print(irrigationTimeMinutes);
     Serial.println("Minuten");
    }
    irrigationTime = irrigationTimeMinutes*60*1000;
    timerTimeLeft =  irrigationTime - (millis() - timerStart);
     if(timerTimeLeft < 0){
      closeAllValves();
      irrigationTimer = "false";
      initialTimerRun = "true";
      client.publish("Garten/Bew/irrigationTimer", String("irrigationTimeStop").c_str());   // gives stop notification over MQTT, ready for new timer start
     }
  }
}

void safety(){
  
//Safety measure: close valves if connection to MQTT or WLAN was interrupted

//Fluss > xyz =>> Alarm "HH Flow" + close all valves
  if ( flowRateLh > 2000 ) {   
        closeAllValves();
        AlarmText = "Flowrate > 1000 L/h, Relais wurden geschlossen";                 // publish alarm text it via MQTT
//        mqttPublish(); Serial.println(AlarmText);
    }

//Fluss < xyz2 (if one or two ports are open) =>> Alarm "LL Flow" 

//Temp > 40°C     Alarm "H Temp"
  if ( t > 40 ) {   
        AlarmText = "Temperatur > 40°C";                                            // publish alarm text it via MQTT
//        mqttPublish(); Serial.println(AlarmText); 
    }
    
//Temp > 50°C     Alarm "HH Temp" + Abschaltung Relais
  if ( t > 50 ) {   
        closeAllValves();
        AlarmText = "Temperatur > 50°C, Relais wurden geschlossen";                 // publish alarm text it via MQTT
//        mqttPublish(); Serial.println(AlarmText);
    }

//Humidity > 60%   Alarm "H Humidity"
  if ( h > 60 ) {   
        AlarmText = "Luftfeuchtigkeit > 60%";                                       // publish alarm text it via MQTT
//        mqttPublish(); Serial.println(AlarmText);
    }
//Humidity > 80%   Alarm "HH Humidity + Abschaltung Relais
  if ( h > 80 ) {   
        closeAllValves();
        AlarmText = "Luftfeuchtigkeit > 80%, Relais wurden geschlossen";            // publish alarm text it via MQTT
//        mqttPublish(); Serial.println(AlarmText);
    }
}


void connect() {
  while(WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.println("WiFi connection failed. Retry.");
  }

  client.setServer(host, 1883);
  client.setCallback(callback);

  if (client.connect("beregnung_relay")) {
    client.subscribe("Garten");
  } else {
    Serial.println("Could not connect to MQTT");
  }  
}

void startbme() {
  //Check BME wiring  
  Serial.println(F("BME280 test"));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

//Auslesen der Wetterdaten
void getWeather() {
    h = bme.readHumidity();
    t = bme.readTemperature();
    dp = t-((100-h)/5);
    p = bme.readPressure()/100.0F;
    dtostrf(t, 5, 1, temperatureString);
    dtostrf(h, 5, 1, humidityString);
    dtostrf(p, 6, 1, pressureString);
    dtostrf(dp, 5, 1, dpString);
}

void getValveStatus() {
  if ( digitalRead(Relay1) ) {Relay1StateString = "relai1off";} else {Relay1StateString = "relai1on";}
  if ( digitalRead(Relay1a) ) {Relay1aStateString = "relai1aoff";} else {Relay1aStateString = "relai1aon";}
  if ( digitalRead(Relay1b) ) {Relay1bStateString = "relai1boff";} else {Relay1bStateString = "relai1bon";}
}

void mqttPublish() {
//    Serial.println("----- Start of MQTT Publishing ------");
    client.publish("Garten/Relay1/state", String(Relay1StateString).c_str());  
    client.publish("Garten/Relay1a/state", String(Relay1aStateString).c_str());
    client.publish("Garten/Relay1b/state", String(Relay1bStateString).c_str());     
    client.publish("Garten/Bew/temp", String(temperatureString).c_str());
    client.publish("Garten/Bew/humidity", String(humidityString).c_str());
    client.publish("Garten/Bew/pressure", String(pressureString).c_str());
    client.publish("Garten/Bew/dewpoint", String(dpString).c_str());
    client.publish("Garten/Bew/flowratelmin", String(flowRateLmin).c_str());   
    client.publish("Garten/Bew/flowratelh", String(flowRateLh).c_str());   
    client.publish("Garten/Bew/volumel", String(totalLitres).c_str());   
//    client.publish("Garten/Bew/volumemicrol", String(totalMicroLitres).c_str());   
    client.publish("Garten/Bew/alarm", String(AlarmText).c_str());   
    client.publish("Garten/Bew/irrigationTimer", String(irrigationTimer).c_str());   // wird zur Bestätigung und Anzeige in Tablet UI verwendet
    client.publish("Garten/Bew/timerTimeLeft", String(timerTimeLeft).c_str());       // wird zur Bestätigung und Anzeige in Tablet UI verwendet
    Serial.println("----- End of MQTT Publishing ------"); 
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(topic);
  Serial.print(" => ");
 
  char* payload_str;
  payload_str = (char*) malloc(length + 1);
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0';
  Serial.println(String(payload_str));
  
  if ( String(topic) == "Garten" ) {
    
    if (String(payload_str) == "relai1on" ) {
      digitalWrite(Relay1, LOW);   // turn the RELAY on
      Relay1StateString = "relai1on"; 
      timeWarning = "start";
    } else if ( String(payload_str) == "relai1off" ) {
      digitalWrite(Relay1, HIGH);    // turn the RELAY off
      Relay1StateString = "relai1off";
      timeWarning = "stopp";
      
    } else if (String(payload_str) == "relai1aon" ) {
      digitalWrite(Relay1a, LOW);   // turn the RELAY on
      Relay1aStateString = "relai1aon";
      timeWarning = "start";
    } else if ( String(payload_str) == "relai1aoff" ) {
      digitalWrite(Relay1a, HIGH);    // turn the RELAY off
      Relay1aStateString = "relai1aoff"; 
      timeWarning = "stopp";
      
    } else if (String(payload_str) == "relai1bon" ) {
      digitalWrite(Relay1b, LOW);   // turn the RELAY on
      Relay1bStateString = "relai1bon"; 
      timeWarning = "start";
    } else if ( String(payload_str) == "relai1boff" ) {
      digitalWrite(Relay1b, HIGH);    // turn the RELAY off
      Relay1bStateString = "relai1boff"; 
      timeWarning = "stopp";
      
    } else if ( String(payload_str) == "obererporton" ) {
      digitalWrite(Relay1a, LOW);    // turn the RELAY on
      delay(500);
      digitalWrite(Relay1, LOW);    // turn the RELAY on
      Relay1StateString = "relai1on";  
      Relay1aStateString = "relai1aon"; 
      timeWarning = "start";

    } else if ( String(payload_str) == "obererportoff" ) {
        if ( digitalRead(Relay1b) ) {   
              digitalWrite(Relay1, HIGH);    // turn the RELAY off
              delay(500);
              digitalWrite(Relay1a, HIGH);    // turn the RELAY off
              Relay1StateString = "relai1off";  
              Relay1aStateString = "relai1aoff"; 
              timeWarning = "stopp";
          } else {
              digitalWrite(Relay1a, HIGH);    // turn the RELAY off
              Relay1aStateString = "relai1aoff";
              timeWarning = "stopp";}

    } else if ( String(payload_str) == "untererporton" ) {
      digitalWrite(Relay1b, LOW);    // turn the RELAY on
      delay(500);
      digitalWrite(Relay1, LOW);    // turn the RELAY on
      Relay1StateString = "relai1on";  
      Relay1bStateString = "relai1bon"; 
      timeWarning = "start"; 
  
    } else if ( String(payload_str) == "untererportoff" ) {
        if ( digitalRead(Relay1a) ) {   
              digitalWrite(Relay1, HIGH);    // turn the RELAY off
              delay(500);
              digitalWrite(Relay1b, HIGH);    // turn the RELAY off
              Relay1StateString = "relai1off";  
              Relay1bStateString = "relai1boff";
              timeWarning = "stopp";
          } else {
              digitalWrite(Relay1b, HIGH);    // turn the RELAY off
              Relay1bStateString = "relai1boff";
              timeWarning = "stopp";}   
                 
    } else if ( String(payload_str) == "irrigationTime15" ) {
      initialTimerRun = "true";
      irrigationTimer = "true";
      irrigationTimeMinutes = 15;

    } else if ( String(payload_str) == "irrigationTime30" ) {
      initialTimerRun = "true";
      irrigationTimer = "true";
      irrigationTimeMinutes = 30;

    } else if ( String(payload_str) == "irrigationTime60" ) {
      initialTimerRun = "true";
      irrigationTimer = "true";
      irrigationTimeMinutes = 60;

    } else if ( String(payload_str) == "irrigationTime90" ) {
      initialTimerRun = "true";
      irrigationTimer = "true";
      irrigationTimeMinutes = 90;
                   
    } else if ( String(payload_str) == "irrigationTimeStop" ) {
      initialTimerRun = "false";
      irrigationTimer = "true";
      irrigationTimeMinutes = 0;
                   
    } else {
      Serial.print("I do not know what to do with ");
      Serial.print(String(payload_str));
      Serial.print(" on topic ");
      Serial.println( String(topic));
    }
  }  
  mqttPublish(); 
}



void getFlowMeterData() {
//    Serial.println("-----------------------------------------------------------------------------------------------");
//    Serial.print("Pulses Counted in this Cycle: ");
//    Serial.println(pulseCount);
    if (pulseCount > 20 || (millis() - oldTime) > 60000) // flow analysis after a minimum of 20 impulses -OR- after 60 seconds elapsed
    {
          //Extracts data input for calculation
          timeDifference = millis() - oldTime;
          countedPulses = pulseCount;
  
          //Reset data input for next calculation
          oldTime = millis();                                                                         //set oldTime to current value
          pulseCount = 0;                                                                             // Reset the pulse counter
          
          //Flows
          flowRateLmin = ((1000.0 / timeDifference) * countedPulses) / calibrationFactor;             //flow rate in L/min
          flowRateLh = flowRateLmin *60;                                                              //flow rate in L/h
  
          //Volumes
          totalMilliLitres = flowRateLmin * timeDifference / 60  + totalMilliLitres;                       //old Milliliter-value + (L/min * 1000 mL/L * (TimeDifference in ms / 1000ms/s / 60s/min))
          totalLitres = totalMilliLitres / 1000;
          totalMicroLitres = totalMilliLitres * 1000;
          Serial.print("timeDifference ");
          Serial.println(timeDifference);
          Serial.print("countedPulses ");
          Serial.println(countedPulses);
          Serial.print("flowRateLmin ");
          Serial.println(flowRateLmin);
          Serial.print("totalMilliLitres ");
          Serial.println(totalMilliLitres);
          Serial.print("totalLitres");
          Serial.println(totalLitres);
          Serial.print("totalMicroLitres");
          Serial.println(totalMicroLitres);
  //        delay(10000);
  
          unsigned int frac;
  
  //        // Print the flow rate for this second in litres / minute
  //        Serial.print("Flow rate: ");
  //        Serial.print(int(flowRateLh)); // Print the integer part of the variable
  //        Serial.println("L/h");
  //        
  //        // Print the cumulative total of litres flowed since starting
  //        Serial.println("Output Liquid Quantity: "); // Output separator
  //        Serial.print(totalMicroLitres);
  //        Serial.println("µL");
  //        Serial.print(totalMilliLitres);
  //        Serial.println("mL");
  //        Serial.print(totalLitres);
  //        Serial.println("L");
  //        Serial.println("-----------------------------------------------------------------------------------------------");  
  }
}


//Interrupt pulseCounter
ICACHE_RAM_ATTR void pulseCounter() {
    pulseCount++;
}

//
//--------------------------------------------------------------------------------------------------------------
//side functions - end ---------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
//
