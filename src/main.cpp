// The MQTT callback function for commands and configuration updates
// Place your message handler code here.
#include <Arduino.h>
#include "universal-mqtt.h"

#include <Wire.h>
#include <WEMOS_SHT3X.h>

#include <ArduinoJson.h>

#define PUBLISH_DELAY 60000
#define CONTROL_DELAY 10000
#define MOTION_DELAY 3000
#define RELAY 13
#define PIR 27
#define LED 26

SHT3X sht30(0x45);

float humidity;
float temperature;
unsigned long lastMillis = 0;
unsigned long lastMillisControl = 0;
unsigned long lastMillisMotion = 0;
bool lastMotion = false;
int relay_state = LOW;
String site = "house.kitchen";

StaticJsonDocument<200> payload;

const String FAIL_TEMP = "Error reading temperature!";

void add_error_to_payload(String error){
  JsonArray data = payload["errors"];
  if(data.isNull()){
    data = payload.createNestedArray("errors");
  }
  
  data.add(error);
}

void update_config(String &topic, String &payload){

  StaticJsonDocument<200> doc;

  DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    String error = error.c_str();
    add_error_to_payload(error);
    return;
  }

  const bool heating_state = doc["heating_state"];
  Serial.print("heating_state: ");
  Serial.println(heating_state ? "TRUE" : "LOW" );
  if(heating_state){
    relay_state = HIGH;
  } else {
    relay_state = LOW;
  }
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  if(topic.endsWith("config")){
    update_config(topic, payload);
  }
}

void relay_control() {
    digitalWrite(RELAY,relay_state);
}

void publish(){
  payload["timestamp"] = time(nullptr);

  String json;
  serializeJson(payload, json);
  publishTelemetry(json);
}

void motion(){
  long state = digitalRead(PIR);
  if(state == HIGH){
    digitalWrite(LED, HIGH);
    payload["motion"] = true;
  } else {
    digitalWrite(LED,LOW);
    payload["motion"] = false;
  }

  if (millis() - lastMillisMotion > MOTION_DELAY
       || state != lastMotion) {
    lastMillisMotion = millis();
    if(state == true){
      Serial.println("MOTION DETECTED");
    }
    if(state == true || lastMotion == true){
      publish();
    }
    lastMotion = state;
  }
}

void loop_control(){
  if (millis() - lastMillisControl > CONTROL_DELAY) {
    lastMillisControl = millis();
    Serial.println("CONTROL");
    relay_control();
  }
}

void loop_publish_temperature(){
  if (millis() - lastMillis > PUBLISH_DELAY) {
    lastMillis = millis();

    if(sht30.get()==0){
      Serial.print("Temperature in Celsius : ");
      Serial.println(sht30.cTemp);
      Serial.print("Temperature in Fahrenheit : ");
      Serial.println(sht30.fTemp);
      Serial.print("Relative Humidity : ");
      Serial.println(sht30.humidity);
      Serial.println();

      payload["temperature"] = sht30.cTemp;
      payload["humidity"] = sht30.humidity;
    }
    else {
      Serial.println(FAIL_TEMP);
      add_error_to_payload(FAIL_TEMP);
      payload.remove("temperature");
      payload.remove("humidity");
    }

    publish();
  }
}

void setup() {
  Serial.begin(115200);

  payload["site"] = site;

  pinMode(RELAY,OUTPUT);
  digitalWrite(RELAY, LOW);

  pinMode(PIR, INPUT);
  pinMode(LED, OUTPUT);

  Wire.begin(); // SDA, SCL

  setupCloudIoT();
}

void loop() {

  mqttClient->loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
  }

  // detect motion
  motion();

  // relay control
  loop_control();

  // publish a message roughly every PUBLISH_DELAY ms.
  loop_publish_temperature();

}

