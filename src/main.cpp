#include <Arduino.h>
#include "WiFi.h"
#include "Zanshin_BME680.h"
#include "PubSubClient.h"

const char* ssid = "MEO-B26010";
const char* pswd = "57ec0c3b1f";

BME680_Class BME680;  

// MQTT
const char* mqtt_url = "192.168.1.75";
const int mqtt_port = 1883;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

const int relay = 26;

void init_wifi(void);
void init_mqtt(void);
void reconnect(void);
void getDataBME(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_wifi();
  init_mqtt();

  pinMode(relay, OUTPUT);

  // Initialize the BME680 sensor
  if (!BME680.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    delay(2000);
  } else {
    Serial.println("BME connected");
  }

  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  digitalWrite(relay, HIGH);
  
  getDataBME(); 
  // digitalWrite(relay, HIGH);
  // Serial.println("Relay OFF");
  
  delay(5000);

  getDataBME();
  // digitalWrite(relay, LOW);
  // Serial.println("Relay ON");

  delay(5000);
}

void init_wifi(void){
  Serial.print("Connecting to Wifi");
  WiFi.begin(ssid, pswd);

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print('.');  
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void init_mqtt(void) {
  client.setServer(mqtt_url, mqtt_port);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("feed_my_plant/status", "Connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void getDataBME(void){
  static int32_t temp, humidity, pressure, gas;

  BME680.getSensorData(temp, humidity, pressure, gas);

  Serial.print("Temperature:");
  Serial.println(temp / 100);

  Serial.print("Humidity:");
  Serial.println(humidity / 1000);

  Serial.print("Pressure:");
  Serial.println(pressure / 100);

  client.publish("feed_my_plant/bme_readings/temp", String(temp / 100).c_str());
  client.publish("feed_my_plant/bme_readings/hum", String(humidity / 1000).c_str());
  client.publish("feed_my_plant/bme_readings/press", String(pressure / 100).c_str());
}
