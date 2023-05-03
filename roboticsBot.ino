#include <WiFi.h>
#include <SR04.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Arduino.h>

//Wifi and MQTT Setup
#define WIFI_SSID "WIFI_SSID_GOES_HERE"
#define WIFI_PASSWORD "WIFI_PASSWORD_GOES_HERE"
//Mosquitto MQTT Server IP and port number
#define MQTT_SERVER "192.168.x.x" 
#define MQTT_PORT 1883


//Pin Assignments
#define DHTPIN 4
#define DHTTYPE DHT11
#define TRIGGER_PIN 21
#define ECHO_PIN 19
//Motor A
#define IN1a 16
#define IN2a 17
#define IN3a 5
#define IN4a 18

//Motor B
#define IN1b 13
#define IN2b 12
#define IN3b 14
#define IN4b 27

const char* MQTT_USERNAME = "Mosquitto_username";
const char* MQTT_PASSWORD = "Mosquitto_password";
float duration = 0.0;
float cm = 0.0;
float inches = 0.0;
float rounded = 0.0;

void callback(char* topic, byte* payload, unsigned int length) {
  // convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // handle incoming message
  switch(message.charAt(0)) {
    case 'F':
      forward();
      break;
    case 'L':
      left();
      break;
    case 'R':
      right();
      break;
    case 'B':
      back();
      break;
    case 'S':
      stop();
      break;
    default:
      break;
  }
}

//Instantiate WiFiClient Object
WiFiClient espClient;
PubSubClient mqtt(MQTT_SERVER, 1883, callback, espClient);

//Instantiate SR04 Ultrasonic sensor
SR04 sr04(TRIGGER_PIN, ECHO_PIN);

//Instantiate dht11 temp object:
DHT dht(DHTPIN, DHTTYPE);

int delayTime = 4;

//Setup
void setup() {
  Serial.begin(115200);
  dht.begin();
  
  //pinMode Declarations
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  //Motor A
  pinMode(IN1a, OUTPUT);
  pinMode(IN2a, OUTPUT);
  pinMode(IN3a, OUTPUT);
  pinMode(IN4a, OUTPUT);
  //Motor B
  pinMode(IN1b, OUTPUT);
  pinMode(IN2b, OUTPUT);
  pinMode(IN3b, OUTPUT);
  pinMode(IN4b, OUTPUT);
  //Connect to WiFi
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
while ((!(WiFi.status() == WL_CONNECTED)))
  {
    delay(300);
    Serial.print("..");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("ESP32 Local IP is : ");
  Serial.print((WiFi.localIP()));
  Serial.print("");
  Serial.println("");
  if (mqtt.connect("ESPClient", "mqtt_username_goes_here", "mqtt_password_goes_here")) {
    mqtt.subscribe("control/motor");
  }
  Serial.println("Subscribe successful!");
}


//function to get Temp and return the value to the call
float getTemperature() {
  float temperature = dht.readTemperature();
  return temperature;
}
//function to get Humidity and return the value to the call
float getHumidity() {
  float humidity = dht.readHumidity();
  return humidity;
}
//function to get distance data from SR04 Ultrasonic sensor
float getDistance(){
  float distance = sr04.Distance();
  return distance;
}
void forward() {
  for (int steps = 0; steps < 400; steps++) {
      //Serial.println("FORWARD");
      forwardMotorA();
      forwardMotorB();
    }
  }

void back() {
  for (int steps = 0; steps < 500; steps++) {
      //Serial.println("BACKWARD");
      backwardMotorA();
      backwardMotorB();
    }
}

void left() {
  for (int steps = 0; steps < 400; steps++) {
      //Serial.println("LEFT");
      backwardMotorA();
      forwardMotorB();
    }
}

void right() {
  for (int steps = 0; steps < 500; steps++) {
      //Serial.println("RIGHT");
      forwardMotorA();
      backwardMotorB();
    }
}

void stop() {
  stopMotors();
}

float measureDistance(){

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
 
//Calculate Distance
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  rounded = round(inches * 100) / 100;
  return rounded;
}


//Reconnect MQTT client if disconnected
void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqtt.connect("ESPClient", "mqtt_username_goes_here", "mqtt_password_goes_here")) {
      Serial.println("connected");
      mqtt.subscribe("control/motor");
    }
  Serial.println("Reconnection Successful!");  
  }
}


void loop() {
  // Reconnect to MQTT broker if necessary
  if (!mqtt.connected()) {
    mqttReconnect();
  }
  
  // Process MQTT messages
  mqtt.loop();
  //Serial.println("loop running");
  // Read sensor values
  float temperature = dht.readTemperature(true);
  float humidity = dht.readHumidity();
  float distance2 = measureDistance();
  
  // Publish sensor values to MQTT broker
  String temperatureStr = String(temperature);
  String humidityStr = String(humidity);
  String distanceStr = String(distance2);
  
  mqtt.publish("sensors/temperature", temperatureStr.c_str());
  mqtt.publish("sensors/humidity", humidityStr.c_str());
  mqtt.publish("sensors/distance", distanceStr.c_str());
  
  // Wait for a short period of time
  delay(500);
}

void forwardMotorA(void) {
  digitalWrite(IN4a, HIGH);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, HIGH);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, HIGH);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, HIGH);
  delay(delayTime);
}

void forwardMotorB(void) {
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, HIGH);
  delay(delayTime);
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, HIGH);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, HIGH);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
  digitalWrite(IN4b, HIGH);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
}

void backwardMotorA(void) {
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, HIGH);
  delay(delayTime);
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, HIGH);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, HIGH);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
  digitalWrite(IN4a, HIGH);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, LOW);
  delay(delayTime);
}

void backwardMotorB(void) {
  digitalWrite(IN4b, HIGH);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, HIGH);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, HIGH);
  digitalWrite(IN1b, LOW);
  delay(delayTime);
  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, HIGH);
  delay(delayTime);
}

void stopMotors(void) {
  digitalWrite(IN4a, LOW);
  digitalWrite(IN3a, LOW);
  digitalWrite(IN2a, LOW);
  digitalWrite(IN1a, LOW);

  digitalWrite(IN4b, LOW);
  digitalWrite(IN3b, LOW);
  digitalWrite(IN2b, LOW);
  digitalWrite(IN1b, LOW);
}
