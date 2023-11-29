#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi network credentials
const char* ssid = "Redmi Note 9 Pro";
const char* password = "asdfgh765";

// ThingsBoard server settings
const char* thingsboardServer = "demo.thingsboard.io";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Pulse Sensor Wire connected to analog pin 0
const int PulseWire = 36;
// Tracks time between pulse readings
long lastPulseTime = 0;
// Measures pulse pin state
int pulseState = LOW;

#define PulseSensor_PIN 36 
#define LED_PIN         23 

int Signal; //--> Accommodates the signal value (ADC value) from the pulse sensor.
// int UpperThreshold = 520;
// int LowerThreshold = 500;

void setup()
{
    Serial.begin(115200); //--> Set's up Serial Communication at certain speed.
    Serial.println();
    delay(2000);

    // Set the ADC resolution. "analogReadResolution(10);" meaning the ADC resolution is set at 10 bits (the ADC reading value is from 0 to 1023).
    analogReadResolution(10);

    // Set LED_PIN as Output.
    pinMode(LED_PIN,OUTPUT);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Connect to ThingsBoard
    client.setServer(thingsboardServer, 1883);
    while (!client.connected()) {
        Serial.println("Connecting to ThingsBoard...");
        if (client.connect("ESP32Client", "CZB0YI9XUFXn64KIEcBb", NULL)) {
            Serial.println("Connected to ThingsBoard");
        } else {
            Serial.print("Failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void loop()
{
    client.loop();

    Signal = analogRead(PulseSensor_PIN); //--> Read the PulseSensor's value. Assign this value to the "Signal" variable.

    Serial.println(Signal); //--> Send the Signal value to Serial.

    // if(Signal > UpperThreshold){ //--> If the signal is above "520"(UpperThreshold), then "turn-on" the LED.
    //   digitalWrite(LED_PIN,HIGH);
    // }

    // if(Signal < LowerThreshold){
    //   digitalWrite(LED_PIN,LOW); //--> Else, the sigal must be below "LowerThreshold", so "turn-off" the LED.
    // }

    if (Signal == HIGH && pulseState == LOW) {
        long pulseDuration = millis() - lastPulseTime;
        long pulseRate = 60000 / pulseDuration;

        Serial.print("Heart rate: ");
        Serial.println(pulseRate);

        lastPulseTime = millis();
        pulseState = HIGH;

        // Send data to ThingsBoard
        String payload = "{";
        payload += "\"heart_rate\":"; payload += String(pulseRate);
        payload += "}";

        client.publish("v1/devices/me/telemetry", (char*) payload.c_str());
    } else if (Signal == LOW) {
        pulseState = LOW;
    }

    delay(20);
}
