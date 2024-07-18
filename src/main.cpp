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

const int RELAY_PIN = 26;


//Functions
void init_wifi(void);
void init_mqtt(void);
void reconnect(void);
void getDataBME(void);
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_wifi();
  init_mqtt();

  pinMode(RELAY_PIN, OUTPUT);

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

  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  getDataBME();   
  delay(10000);
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
      client.publish("office_stats/status", "Connected");
      // Resubscribe to the topic after reconnecting
      client.subscribe("office_stats/relay");
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

  client.publish("office_stats/bme_readings/temp", String(temp / 100).c_str());
  client.publish("office_stats/bme_readings/hum", String(humidity / 1000).c_str());
  client.publish("office_stats/bme_readings/press", String(pressure / 100).c_str());
}

void callback(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
        Serial.print((char)payload[i]);
    Serial.println();

    if (!strcmp(topic, "office_stats/relay"))
    {
        if ((char)payload[0] == '1'){
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Relay On");
        }
        else{
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("Relay OFF");          
        }
    }
    
}