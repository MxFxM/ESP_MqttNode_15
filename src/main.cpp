#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>

#include <SFE_BMP180.h>
#include <Wire.h>

#define ALTITUDE 190.0 // Altitude of Oberderdingen

#include "credentials.h"

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 600

#define BATTERY_PIN 34
#define ADC_RESOLUTION 4095

#define REED_PIN 4

#define ESP_SENSOR_NR 1

WiFiClient espClient;
PubSubClient client(espClient);

SFE_BMP180 pressure;

void setup_wifi(void);
// void callback(char *topic, byte *message, unsigned int length);
void reconnect(void);

unsigned long lastMillis;
int reconnectCounter;

long lastMsg = 0;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    uint8_t reed_state = digitalRead(REED_PIN);

    setup_wifi();

    // Serial.println(WiFi.getTxPower());

    client.setServer(mqtt_broker, 1883);
    // client.setCallback(callback);

    do
    {

        pinMode(23, OUTPUT);
        digitalWrite(23, HIGH);

        pinMode(BATTERY_PIN, INPUT);

        pinMode(REED_PIN, INPUT_PULLUP);

        if (!client.connected())
        {
            reconnect();
        }
        // client.loop();

        // long now = millis();
        // if (now - lastMsg > 5000)
        //{
        //   lastMsg = now;
        Serial.println("Hello World");
        double T, P, p0;

        reed_state = digitalRead(REED_PIN);

        if (pressure.begin())
        {
            Serial.println("BMP180 init success");

            char status;

            // You must first get a temperature measurement to perform a pressure reading.

            // Start a temperature measurement:
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.
            status = pressure.startTemperature();
            if (status != 0)
            {
                // Wait for the measurement to complete:
                delay(status);

                // Retrieve the completed temperature measurement:
                status = pressure.getTemperature(T);
                if (status != 0)
                {
                    // Print out the measurement:
                    Serial.print("temperature: ");
                    Serial.print(T, 2);
                    Serial.println(" deg C");

                    // Start a pressure measurement:
                    // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
                    status = pressure.startPressure(3);
                    if (status != 0)
                    {
                        // Wait for the measurement to complete:
                        delay(status);

                        // Retrieve the completed pressure measurement:
                        status = pressure.getPressure(P, T);
                        if (status != 0)
                        {
                            // Print out the measurement:
                            Serial.print("absolute pressure: ");
                            Serial.print(P, 2);
                            Serial.println(" hPa");

                            // To remove the effects of altitude, use the sealevel function and your current altitude.
                            // This number is commonly used in weather reports.
                            // Result: p0 = sea-level compensated pressure in mb
                            p0 = pressure.sealevel(P, ALTITUDE);
                            Serial.print("relative (sea-level) pressure: ");
                            Serial.print(p0, 2);
                            Serial.println(" hPa");
                        }
                        else
                        {
                            Serial.println("error retrieving pressure measurement\n");
                        }
                    }
                    else
                    {
                        Serial.println("error starting pressure measurement\n");
                    }
                }
                else
                {
                    Serial.println("error retrieving temperature measurement\n");
                }
            }
            else
            {
                Serial.println("error starting temperature measurement\n");
            }
        }
        else
        {
            // Oops, something went wrong, this is usually a connection problem,
            // see the comments at the top of this sketch for the proper connections.

            Serial.println("BMP180 init fail\n\n");
        }

        // get an analog voltage between 0 and 3.3v
        // due to the voltage divider, the value has to be doubled for the battery voltage
        double battery_value = ((float)analogRead(BATTERY_PIN) / (float)ADC_RESOLUTION) * 3.3 * 2;
        Serial.print("Battery level: ");
        Serial.print(battery_value);
        Serial.println(" V");

        //  double T, P, p0;

        char message[50];
        // dtostrf(battery_value, 1, 2, message);
        sprintf(message, "%d %f %f %f %f, %d", ESP_SENSOR_NR, battery_value, T, P, p0, reed_state);
        client.publish("esp32", message);
        client.loop();
        // need to wait until packet is sent
        // how cand we limit this time?
        // 10s is save, but too long
        // i go for 2 seconds for now
        delay(2000);

    } while (reed_state != digitalRead(REED_PIN));

    Serial.flush();

    // nicely disconnect from wifi before going to sleep
    WiFi.disconnect();

    digitalWrite(23, LOW);

    // 1 minute interval
    // adjust sleeping time for time it took to connect to the wlan
    // micros should restart every time we wake up from deep sleep
    // that can be used to check how long the connection took
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)REED_PIN, !digitalRead(REED_PIN));
    esp_deep_sleep_start();

    Serial.println("There was a problem if this prints");
}

void loop()
{
    // put your main code here, to run repeatedly:
    // this is never reached
}

void setup_wifi()
{
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    lastMillis = millis();
    reconnectCounter = 0;

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        // after 5 seconds, attempt by disconnecting and restarting wifi
        if ((millis() - lastMillis) >= 5000)
        {
            Serial.println("attempting disconnect");
            reconnectCounter++;
            // after 3 restart attempts (15s), ignore the measurement and goto sleep
            if (reconnectCounter >= 3)
            {
                Serial.println("giving up");
                esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
                esp_deep_sleep_start();
            }

            WiFi.disconnect();
            WiFi.begin(ssid, password);
            lastMillis = millis();
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
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
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32"))
        {
            Serial.println("connected");
            // Subscribe
            client.subscribe("esp32/output");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}