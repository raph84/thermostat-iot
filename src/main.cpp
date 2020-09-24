// The MQTT callback function for commands and configuration updates
// Place your message handler code here.
#include <Arduino.h>
#include "universal-mqtt.h"

#include <Wire.h>
#include <SPI.h>

#include <ArduinoJson.h>

#define PUBLISH_DELAY 60000
#define CONTROL_DELAY 10000
#define RELAY 13

float humidity;
float temperature;
unsigned long lastMillis = 0;
unsigned long lastMillisControl = 0;
int relay_state = LOW;



void update_config(String &topic, String &payload){

  StaticJsonDocument<200> doc;

  DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
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

void setup() {
  Serial.begin(115200);

  pinMode(RELAY,OUTPUT);
  digitalWrite(RELAY, LOW);

  Wire.begin(15, 2); // SDA, SCL

  setupCloudIoT();
}

void loop() {
  mqttClient->loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
  }

  if (millis() - lastMillisControl > CONTROL_DELAY) {
    lastMillisControl = millis();
    Serial.println("CONTROL");
    relay_control();
  }

  // publish a message roughly every PUBLISH_DELAY ms.
  if (millis() - lastMillis > PUBLISH_DELAY) {
    lastMillis = millis();

    temperature = 1.0;
    humidity = 1.0;

    String payload = String("{\"timestamp\":") + time(nullptr) +
                     String(",\"temperature\":") + temperature +
                     String(",\"humidity\":") + humidity +
                     String("}");
    publishTelemetry(payload);

    
  }

}

