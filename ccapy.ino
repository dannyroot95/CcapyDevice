#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h >
#include <WiFiManager.h>
#include <Ticker.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>

#include <FirebaseArduino.h>
#include <ArduinoJson.h>

// Instance from ticker class
Ticker ticker;
// Pin LED blue
byte pinLed = D3;
unsigned long currentMillis = 60000;

void changeLed() {
  // Change status led
  byte statusLed = digitalRead(pinLed);
  digitalWrite(pinLed, !statusLed);
}


#if defined(ESP32)
#error "Software Serial is not supported on the ESP32"
#endif

/* Use software serial for the PZEM
   Pin 12 Rx (Connects to the Tx pin on the PZEM)
   Pin 13 Tx (Connects to the Rx pin on the PZEM)
*/
#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 12
#define PZEM_TX_PIN 13
#endif

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

#define FIREBASE_HOST "ccapy-2bc76-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "eV029Acw39GJNRudc7Im3RIkw9Nil1nkQnDlYIBQ"

const int inputPin = 4;
byte inputPinCounter = D2;
byte ledPIN = D8;
byte ledPINSEND = D10;
int value = 0;
int value2 = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(inputPin, INPUT);
  pinMode(inputPinCounter, INPUT);
  pinMode(ledPIN , OUTPUT);
  pinMode(ledPINSEND , OUTPUT);
  pinMode(pinLed, OUTPUT);

  ticker.attach(0.2, changeLed);

  // Create a instance from WifiManager class
  WiFiManager wifiManager;

  if (!wifiManager.autoConnect("CCAPY")) {
    Serial.println("Fallo en la conexión (timeout)");
    ESP.reset();
    delay(1000);
  }

  Serial.println("Succes conecct");
  // delete timer
  ticker.detach();

  // off LED
  digitalWrite(pinLed, LOW);

  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(0);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
 
}

void loop() {

  resetSettingsCappy();

  if (WiFi.status() == WL_CONNECTED)
  {
    while (WiFi.status() == WL_CONNECTED) {

  Serial.print("Custom Address:");
    Serial.println(pzem.readAddress(), HEX);
    
      float voltage = pzem.voltage();
      float current = pzem.current();
      float power = pzem.power();
      float energy = pzem.energy();
      float frequency = pzem.frequency();
      float pf = pzem.pf();
      String id = String(ESP.getChipId());

      timeClient.update();
      time_t epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime ((time_t *)&epochTime); 
      int monthDay = ptm->tm_mday;
  
      String dateDay = Firebase.getString("devices/"+id+"/config/day_invoice");
      int index = dateDay.indexOf('/');
      String days = dateDay.substring(0,index);
      int day_invoice = days.toInt();
      String resetEnergy = Firebase.getString("devices/"+id+"/config/is_reset");
  

      if (isnan(voltage)) {
        Serial.println("Error reading voltage");
        resetSettingsCappy();
        digitalWrite(pinLed , HIGH);   // poner el Pin en HIGH
        delay(200);                   // esperar un segundo
        digitalWrite(pinLed , LOW);    // poner el Pin en LOW
        delay(200);
      } else if (isnan(current)) {
        Serial.println("Error reading current");
      } else if (isnan(power)) {
        Serial.println("Error reading power");
      } else if (isnan(energy)) {
        Serial.println("Error reading energy");
      } else if (isnan(frequency)) {
        Serial.println("Error reading frequency");
      } else if (isnan(pf)) {
        Serial.println("Error reading power factor");
      } else {

        // Print the values to the Serial console

        Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
        Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
        Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
        Serial.print("Energy: ");       Serial.print(energy);     Serial.println("kWh");
        Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
        Serial.print("PF: ");           Serial.println(pf);
        Serial.print("CHIP ID: ");      Serial.println(ESP.getChipId());
        Serial.print("Time: ");         Serial.println(epochTime);
       
        currentMillis = currentMillis + 1000;

        if (currentMillis >= 60000) {

          StaticJsonBuffer<256> jsonBuffer;
          JsonObject& root = jsonBuffer.createObject();
          root["voltage"] = voltage;
          root["ampere"] = current;
          root["watts"] = power;
          root["energy"] = energy;
          root["frequency"] = frequency;
          root["power_factor"] = pf;
          root["chip_id"] = ESP.getChipId();
          root["time"] = epochTime;
          
          Firebase.setFloat("devices/" + id + "/current_data/voltage", voltage);
          delay(1000);
          Firebase.setFloat("devices/" + id + "/current_data/ampere", current);
          delay(1000);
          Firebase.setFloat("devices/" + id + "/current_data/watts", power);
          delay(1000);
          Firebase.setFloat("devices/" + id + "/current_data/energy", energy);
          delay(1000);
          Firebase.setFloat("devices/" + id + "/current_data/frequency", frequency);
          delay(1000);
          Firebase.setFloat("devices/" + id + "/current_data/power_factor", pf);
          delay(1000);
          Firebase.setInt("devices/" + id + "/current_data/chip_id", ESP.getChipId());
          delay(1000);
          Firebase.setInt("devices/" + id + "/current_data/time", epochTime);
          delay(1000);
          Firebase.push("devices/" + id + "/registers", root);

          currentMillis = 0;

          //if(monthDay >= day_invoice){
           // Serial.println("Reset");
              if(resetEnergy == "true"){
                  pzem.resetEnergy();
                  //Firebase.setString("devices/"+id+"/config/is_reset","true");
                  Firebase.setInt("devices/"+id+"/config/counter",1);
                  Firebase.setString("devices/"+id+"/config/is_reset","false");
                  //}
                  }else{
                    //Firebase.setString("devices/"+id+"/config/is_reset","false");
                    }
          
          digitalWrite(ledPINSEND , HIGH);   // poner el Pin en HIGH
          delay(200);                   // esperar un segundo
          digitalWrite(ledPINSEND , LOW);    // poner el Pin en LOW
          delay(200);     
          digitalWrite(ledPINSEND , HIGH);   // poner el Pin en HIGH
          delay(200);                   // esperar un segundo
          digitalWrite(ledPINSEND , LOW); 
          delay(200);  
          digitalWrite(ledPINSEND , HIGH);   // poner el Pin en HIGH
          delay(200);                   // esperar un segundo
          digitalWrite(ledPINSEND , LOW);
        }
        Serial.println();
        delay(1000);
      }


    }
  }
  else {
    Serial.println("Not conecction!");
    delay(1000);
  }

}

void resetSettingsCappy() {

  WiFiManager wifiManager;
  value = digitalRead(inputPin);  //lectura digital de pin
  value2 = digitalRead(inputPinCounter);

  //mandar mensaje a puerto serie en función del valor leido
  if (value != HIGH) {
    Serial.println("Reset");
    digitalWrite(ledPIN , HIGH);   // poner el Pin en HIGH
    delay(200);
    wifiManager.resetSettings();
    delay(1000);
    digitalWrite(ledPIN , LOW);    // poner el Pin en LOW
    delay(200);
    digitalWrite(ledPIN , HIGH);   // poner el Pin en HIGH
    delay(200);
    digitalWrite(ledPIN , LOW);    // poner el Pin en LOW
    delay(200);
    digitalWrite(ledPIN , HIGH);   // poner el Pin en HIGH
    delay(200);                   // esperar un segundo
    digitalWrite(ledPIN , LOW);    // poner el Pin en LOW
    delay(200);
    ESP.reset();
    delay(2);
  }

if (value2 != HIGH) {
    Serial.println("Counter reset");
    digitalWrite(pinLed , HIGH);   // poner el Pin en HIGH
    delay(200);
    pzem.resetEnergy();
    delay(2000);
    digitalWrite(pinLed , LOW);    // poner el Pin en LOW
    delay(200);
    digitalWrite(pinLed , HIGH);   // poner el Pin en HIGH
    delay(200);
    digitalWrite(pinLed , LOW);    // poner el Pin en LOW
    delay(200);
    digitalWrite(pinLed , HIGH);   // poner el Pin en HIGH
    delay(200);                   // esperar un segundo
    digitalWrite(pinLed , LOW);    // poner el Pin en LOW
    delay(200);
  }


}
