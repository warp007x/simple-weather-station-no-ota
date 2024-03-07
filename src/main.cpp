#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


//TaskHandle_t main_loop;
//TaskHandle_t ota_task;


//void mainLoop(void *pvParameters);
//void otaUpdate(void *pvParameters);

#define DHTTYPE DHT22

EspSoftwareSerial::UART mod;

//  Sensor Pin Map
#define DHTPIN 10
#define WIND_SPEED 8
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_ENB 13
#define MYPORT_TX 41
#define MYPORT_RX 42

TwoWire i2c_0 = TwoWire(0);

Adafruit_BME680 bme(&i2c_0);


//  LED Pins
const int power_led = 11;
const int msg_status = 12;

// Replace the next variables with your SSID/Password combination
const char* ssid[3] = {"IEMA IOT" ,"WARP", "wifi2"};
const char* password[3] = {"Iot@iema_23", "ankg2279", "pass2"};

const char* topic = "IEMA/WST/";
char device_id[17] = "";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedDate;
String ota_time1_1 = "19:00:00";
String ota_time1_2 = "19:00:01";
String ota_time1_3 = "19:00:02";

String ota_time2_1 = "03:00:00";
String ota_time2_2 = "03:00:01";
String ota_time2_3 = "03:00:02";

String ota_time3_1 = "11:00:00";
String ota_time3_2 = "11:00:01";
String ota_time3_3 = "11:00:02";

const char *mqtt_username = "iema_iot";
const char *mqtt_password = "12345";
// Add your MQTT Broker IP address, example:
// const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_server = "192.168.50.9";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// Parameters
float temperature   = 0.0;
float temperature1  = 0.0;
float humidity      = 0.0;
float humidity1     = 0.0;
float pressure      = 0.0;
float air_quality   = 0.0;
int windspeed       = 0.0;
String dayStamp;
String timeStamp;
bool sensorReadSuccessful = false;

DHT dht(DHTPIN, DHTTYPE);

RTC_DATA_ATTR int bootCount = 0;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup_wifi() {
  delay(10);
  bool connected_status = false;
  int i = 0;
  // We start by connecting to a WiFi network
  Serial.println();
  while(!connected_status){
    int k = 0;
    bool flag = false;
    Serial.println("Connecting to ");
    Serial.println(ssid[i]);
    Serial.println("Password ");
    Serial.println(password[i]);
    WiFi.begin(ssid[i], password[i]);
  
    while (WiFi.status() != WL_CONNECTED) {
//    delay(300);
        digitalWrite(msg_status, LOW);
        Serial.print(".");
        digitalWrite(msg_status, HIGH);
        delay(200); 
        digitalWrite(msg_status, LOW);
        delay(300);
        if (k >= 20){
          flag = true;
          break;
        } 
        k++;  
  }
  if(flag){
    if (i>=2){
      Serial.println("Couldn't connet to WiFi!, trying again.");
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, LOW);
      digitalWrite(msg_status, HIGH);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, LOW);
      digitalWrite(msg_status, HIGH);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, LOW);
      digitalWrite(msg_status, HIGH);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, LOW);
      digitalWrite(msg_status, HIGH);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, HIGH);
      digitalWrite(msg_status, LOW);
      delay(200);
      digitalWrite(power_led, LOW);
      digitalWrite(msg_status, LOW);
      delay(200);
      i = 0;
      continue;
      }
    i++;
    continue;
  }
  connected_status = true;
  Serial.println("");
  Serial.printf("\nConnected to WiFi %s\n", ssid[i]);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Device MAC address: ");
  String wifiMac = WiFi.macAddress();
  Serial.println(wifiMac);
  strcpy(device_id, wifiMac.c_str());
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
   Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "IEMA/NODE/0xFF") {
    Serial.println(messageTemp);
    }
  }


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    String client_id = "iema-esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.printf("\nConnected to: %s \n", mqtt_server);
      digitalWrite(power_led, HIGH);
      // Subscribe
      client.subscribe("IEMA/NODE/0xFF");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      digitalWrite(power_led, HIGH);
      Serial.println(" trying again in 5 seconds...");
      // Wait 5 seconds before retrying
      digitalWrite(power_led, LOW);
      delay(500);
      digitalWrite(power_led, HIGH);
      delay(500);
      digitalWrite(power_led, LOW);
      delay(500);
      digitalWrite(power_led, HIGH);
      delay(500);
      digitalWrite(power_led, LOW);
      delay(500);
      digitalWrite(power_led, HIGH);
      delay(500);
      digitalWrite(power_led, LOW);
      delay(500);
      digitalWrite(power_led, HIGH);
      delay(500);
      digitalWrite(power_led, LOW);
      delay(500);
      digitalWrite(power_led, HIGH);
      delay(500);
      digitalWrite(power_led, LOW);
      
    }
  }
}

int calc_windSpeed(){
  
   int sensorValue = analogRead(WIND_SPEED);
   float outvoltage = sensorValue * (5.0 / 1023.0);
//   Serial.print("outvoltage = ");
//   Serial.print(outvoltage);
//   Serial.println("V");
   int speed_anemo = map(outvoltage, 0.0, 10.0, 0.0, 40.0); //The level of wind speed is proportional to the output voltage.
//   Serial.print("wind speed is ");
//   Serial.print(speed_anemo);
//   Serial.println(" m/s");
//   Serial.println();
//   delay(500);
    return speed_anemo;
  
  }

void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  temperature1 = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity1 = bme.humidity;
  air_quality = bme.gas_resistance / 1000.0;
}


void updateTimeGMT(){
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  formattedDate = timeClient.getFormattedDate();
  // Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  // Serial.print("DATE: ");
  // Serial.println(dayStamp);
  // Extract time
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  // Serial.print("HOUR: ");
  // Serial.println(timeStamp);
  delay(100);
}

void setup() {

//  xTaskCreatePinnedToCore(
//                    mainLoop,    /* Task function. */
//                    "loop",      /* name of task. */
//                    10000,       /* Stack size of task */
//                    NULL,        /* parameter of the task */
//                    1,           /* priority of the task */
//                    &main_loop,  /* Task handle to keep track of created task */
//                    1);          /* pin task to core 0 */                  
//  delay(500); 
//
//  
//  xTaskCreatePinnedToCore(
//                    otaUpdate,   /* Task function. */
//                    "OTA",       /* name of task. */
//                    10000,       /* Stack size of task */
//                    NULL,        /* parameter of the task */
//                    1,           /* priority of the task */
//                    &ota_task,   /* Task handle to keep track of created task */
//                    0);          /* pin task to core 1 */
//    delay(500); 
  
  pinMode(power_led, OUTPUT);
  pinMode(msg_status, OUTPUT);
  pinMode(I2C_ENB, OUTPUT);

  digitalWrite(I2C_ENB, HIGH);
  delay(100);
  
  i2c_0.begin(I2C_SDA, I2C_SCL, 100000);
  delay(100);
  Serial.begin(115200);

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);


    mod.begin(4800, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
      if (!mod) { // If the object did not initialize, then its configuration is invalid
          Serial.println("Invalid EspSoftwareSerial pin configuration, check config"); 
          while (1) { // Don't continue with invalid configuration
          delay (1000);
          }
      } 

  dht.begin();
  bool status;
  status = bme.begin(); 
  
  if (!status) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    delay(5000);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  digitalWrite(power_led, HIGH);
  delay(1000);
  digitalWrite(power_led, LOW);
  delay(1000);
  digitalWrite(msg_status, HIGH);
  delay(1000);
  digitalWrite(msg_status, LOW);
  delay(1000);
  digitalWrite(power_led, HIGH);
  delay(1000);
  setup_wifi();
  delay(1000);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(3600);

  digitalWrite(power_led, HIGH);
  delay(1000);
  digitalWrite(power_led, LOW);
  delay(1000);
  digitalWrite(power_led, HIGH);
  delay(1000);
  digitalWrite(power_led, LOW);
  delay(1000);
  digitalWrite(power_led, HIGH);
  delay(1000);
  
}  

void loop() {
    updateTimeGMT();
    delay(100);
    if(timeStamp == ota_time1_1 || timeStamp == ota_time1_2 || timeStamp == ota_time1_3
        || timeStamp == ota_time2_1 || timeStamp == ota_time2_2 || timeStamp == ota_time2_3 
        || timeStamp == ota_time3_1 || timeStamp == ota_time3_2 || timeStamp == ota_time3_3){
    Serial.println("Going to sleep now");
    delay(1000);
    Serial.flush();
    delay(3000);
    esp_deep_sleep_start();
    }
    delay(1000);
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    int resultLength = strlen(topic) + strlen(device_id) + 1;
    char* mqtt_topic = new char[resultLength];
    strcpy(mqtt_topic, topic);
    strcat(mqtt_topic, device_id);
    long now = millis();
    if (now - lastMsg > 4000) {
              lastMsg = now;
              getBME680Readings();
          
              float newTemperature = dht.readTemperature();
              float newHumidity = dht.readHumidity();
          
              if (!isnan(newTemperature) && !isnan(newHumidity)) {
                    temperature = newTemperature;
                    humidity = newHumidity;
                    sensorReadSuccessful = true;
                   } 
              else {
                    temperature = temperature1;
                    humidity = humidity1;
                    sensorReadSuccessful = false;
                   }

              windspeed = calc_windSpeed();
              
              StaticJsonDocument<1000> JSONbuffer;
              JsonObject JSONencoder = JSONbuffer.createNestedObject();
             
              JSONencoder["device"]         = device_id;
              JSONencoder["temperature"]    = roundf(temperature*100)/100.0;
              JSONencoder["humidity"]       = roundf(humidity*100)/100.0;
              JSONencoder["pressure"]       = roundf(pressure*100)/100.0;
              JSONencoder["air_quality"]    = air_quality;
              JSONencoder["pm_2_5"]         = 0.0;
              JSONencoder["pm_10"]          = 0.0;
              JSONencoder["wind_speed"]     = windspeed;

              
           
              char JSONmessageBuffer[1000];
              serializeJson(JSONencoder, JSONmessageBuffer);
              Serial.println("Sending message to MQTT topic..");
              Serial.print(mqtt_topic);
              Serial.println(JSONmessageBuffer);
              
              if (client.publish(mqtt_topic, JSONmessageBuffer, 0) == true) {
                  Serial.println("Success sending message");
                    digitalWrite(msg_status, HIGH);
                    delay(70);
                    digitalWrite(msg_status, LOW);
                    delay(100);
                } 
              else {
                  Serial.println("Error sending message");
                  digitalWrite(msg_status, LOW);
            }
            }
}