// The MQTT callback function for commands and configuration updates
// Place your message handler code here.
#include <Arduino.h>

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

#include "universal-mqtt.h"

#include <Wire.h>
#include <SPI.h>


#define PUBLISH_DELAY 60000

float humidity;
float temperature;
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin(15, 2); // SDA, SCL

  setupCloudIoT();
}

void loop() {
  mqttClient->loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
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