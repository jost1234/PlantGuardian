#include <Adafruit_AHTX0.h>
#include "stdint.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // Install PubSubClient by Nick O'Leary

// LED for alarm intervention
const int LED_PIN = LED_BUILTIN;  // the number of the LED pin
uint8_t alarm = 0;

// Variables will change:
int ledState = LOW;  // ledState used to set the LED
bool actionNeeded = false;

// Light sensitive resistor
const int LIGHT_SENSOR_PIN = A0;
int16_t light;
char lightHours[24] = {0};  // To measure how many light hour were today
char lightHourCurrent = 0;
bool lightHourSendReady = false;

#define lightSampleCount 12
#define lightSampleThreshold 70
char lightSamples[lightSampleCount] = {0};
char lightSampleCurrent = 0;

// Temperature and humidity sensor
Adafruit_AHTX0 aht;
sensors_event_t humidity, temp;

// Multitasking Timers
unsigned long previousMillis = 0;
namespace Timer
{
  // Light scan
  uint32_t lightScanInt = 1000 * 60 * 5;  // 5 minutes
  uint32_t lightScanPrevTime = 0;

  // Humidity and Temperature sensor scan
  uint32_t ahtScanInt = 1000 * 60;
  uint32_t ahtScanPrevTime = 0;

  // Blinker LED period
  uint32_t LEDblinkInt = 1000;
  uint32_t LEDblickPrevTime = 0;
}

// WiFi and MQTT stuff

// WiFi
const char* ssid = "ENTER-SSID-HERE";
const char* password = "ENTER-PASSWORD-HERE";
const char* mqtt_server = "192.168.137.1"; // Mobile hotspot from laptop
const char* mqtt_port = "1883";

// MQTT Topics
#define MQTT_PUB "paper_wifi/test/flower_monitor"
#define MQTT_ALARM "paper_wifi/test/alarm"

WiFiClient espClient;
PubSubClient client(espClient);

// Function to reconnect to MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      client.subscribe(MQTT_ALARM);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// To catch alarm signal
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] with payload: ");
  for (unsigned int i = 0; i < length; i++) {
     Serial.print((char)payload[i]);
  }
  Serial.println();
  
  if (strcmp(topic, MQTT_ALARM) == 0) {
    alarm = atoi((char*)payload);
  }
}

void setup() {
  // put your setup code here, to run once:

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("");   // Better to look at after reset

  pinMode(LED_PIN, OUTPUT);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  pinMode(LIGHT_SENSOR_PIN, INPUT);

  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(1000);
  }
  Serial.println("AHT10 found");

  // connecting to a WiFi network
  setup_wifi();
  // connecting to MQTT server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // Round robin architecture
  // - read temperature and humidity
  // - read light data
  // - in case of action nedded: LED ON
  // - write serial monitor
  // - send data in MQTT
  unsigned long currentMillis = millis();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // MQTT Buffer
  char str[50];

  if(currentMillis - Timer::ahtScanPrevTime >= Timer::ahtScanInt)
  {
    Timer::ahtScanPrevTime = currentMillis;

    // Temperature sensor read logic - read AHT10 and send data via MQTT
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    //Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

    // Publish an MQTT message about temperature and humidity
    
    sprintf(str, "{\"temperature\":%.2f, \"humidity\":%.2f}", temp.temperature, humidity.relative_humidity);
    client.publish(MQTT_PUB, str);
    Serial.printf("Publishing on topic %s: ", MQTT_PUB);
    Serial.println(str);

  }

  if(currentMillis - Timer::lightScanPrevTime >= Timer::lightScanInt)
  {
    Timer::lightScanPrevTime = currentMillis;

    // Light sensor read logic every 5 minutes
    lightSampleCurrent++; // Cyclic increment
    lightSampleCurrent %= 12;

    /// Read analog A0 pin, if value > THRESHOLD : Light
    light = analogRead(LIGHT_SENSOR_PIN);
    light = map(light, 0, 1023, 0, 100); // map the value from 0, 1023 to 0, 100
    //Serial.print("Light: "); Serial.print(light); Serial.println("%");
    sprintf(str, "{\"light\":%d}", light);
    client.publish(MQTT_PUB, str);
    Serial.printf("Publishing on topic %s: ", MQTT_PUB);
    Serial.println(str);

    lightSamples[lightSampleCurrent] = (light >= lightSampleThreshold);

    // Every hour send data
    if(lightSampleCurrent == 0)
    {
      int lightValue = 0;
      for(int i=0; i<12; i++) // Summarize the array into lightValue
        if(lightSamples[i]) lightValue++;
      
      // Cyclic increment
      lightHourCurrent++;
      lightHourCurrent %= 24;

      lightHours[lightHourCurrent] = (lightValue >= 8);
      int lightHourValue = 0;
      for(int i=0; i<24; i++) // Summarize the array into lightHourValue
        if(lightHours[i]) lightHourValue++;

      Serial.print("Light Hours: "); Serial.println(lightHourValue);

      // Wait 24 before sending data
      if(lightHourCurrent == 0 && !lightHourSendReady)
      {
        lightHourSendReady = true;
        Serial.print("Light Hours data ready to send! ");
      }

      if(lightHourSendReady)
      {
        // Send Light Hours data via MQTT
        sprintf(str, "{\"light_hours\":%d}", lightHourValue);
        client.publish(MQTT_PUB, str);
        Serial.printf("Publishing on topic %s: ", MQTT_PUB);
        Serial.println(str);
      }
    }
  }
  
  if(alarm && currentMillis - Timer::LEDblickPrevTime >= Timer::LEDblinkInt)
  {
    Timer::LEDblickPrevTime = currentMillis;

    if(ledState == LOW){
      ledState = HIGH;
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      ledState = LOW;
      digitalWrite(LED_PIN, HIGH);
    }
  }
  
  if(!alarm && ledState == HIGH){
    ledState = LOW;
    digitalWrite(LED_PIN, HIGH);
  }
  

}
