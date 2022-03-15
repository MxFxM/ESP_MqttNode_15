#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>

#include <SFE_BMP180.h>
#include <Wire.h>

#include "credentials.h"

#define BATTERY_PIN1 34
#define BATTERY_PIN2 35
#define BATTERY_PIN3 36
#define ADC_RESOLUTION 4095

#define REED_PIN 4

#define ESP_SENSOR_NR 1

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi(void);
// void callback(char *topic, byte *message, unsigned int length);
void reconnect(void);

unsigned long lastMillis;
int reconnectCounter;

long lastMsg = 0;

void setup()
{
    setup_wifi();

    client.setServer(mqtt_broker, 1883);

    pinMode(BATTERY_PIN1, INPUT);
    pinMode(BATTERY_PIN2, INPUT);
    pinMode(BATTERY_PIN3, INPUT);
}

void loop()
{
    // put your main code here, to run repeatedly:
    // this is never reached

    if (!client.connected())
    {
        reconnect();
    }

    // get an analog voltage between 0 and 3.3v
    // due to the voltage divider, the value has to be doubled for the battery voltage
    double battery_value1 = ((float)analogRead(BATTERY_PIN1) / (float)ADC_RESOLUTION) * 3.3 * 2;
    double battery_value2 = ((float)analogRead(BATTERY_PIN2) / (float)ADC_RESOLUTION) * 3.3 * 2;
    double battery_value3 = ((float)analogRead(BATTERY_PIN3) / (float)ADC_RESOLUTION) * 3.3 * 2;

    char message[50];
    // dtostrf(battery_value, 1, 2, message);
    sprintf(message, "%d %f %f %f", ESP_SENSOR_NR, battery_value1, battery_value2, battery_value3);
    client.publish("esp32/bat", message);
    client.loop();

    delay(10000);
}

void setup_wifi()
{
    delay(10);
    WiFi.begin(ssid, password);

    lastMillis = millis();
    reconnectCounter = 0;

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);

        // after 5 seconds, attempt by disconnecting and restarting wifi
        if ((millis() - lastMillis) >= 5000)
        {
            WiFi.disconnect();
            WiFi.begin(ssid, password);
            lastMillis = millis();
        }
    }
}

/*
void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  if (String(topic) == "esp32/output")
  {
  }
}
*/

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        // Attempt to connect
        if (client.connect("ESP32"))
        {
            // Subscribe
            client.subscribe("esp32/output");
        }
        else
        {
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}