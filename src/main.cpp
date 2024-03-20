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
#include "DFRobot_AirQualitySensor.h"
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#define I2C_ADDRESS_PM    0x19
  


//TaskHandle_t main_loop;
//TaskHandle_t ota_task;


//void mainLoop(void *pvParameters);
//void otaUpdate(void *pvParameters);

#define DHTTYPE DHT22

EspSoftwareSerial::UART mod;

//  Sensor Pin Map
#define DHTPIN 33
#define WIND_SPEED 10
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  240        /* Time ESP32 will go to sleep (in seconds) */
#define SEALEVELPRESSURE_HPA (1013.25)
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_ENB 13
#define MYPORT_TX 41
#define MYPORT_RX 42

TwoWire i2c_0 = TwoWire(0);
WebServer server(80);

Adafruit_BME680 bme(&i2c_0);
DFRobot_AirQualitySensor particle(&i2c_0 ,I2C_ADDRESS_PM);


//  LED Pins
const int power_led  = 39;
const int msg_status = 40;

// Replace the next variables with your SSID/Password combination
const char* host = "iema_iot";
const char* ssid[3] = {"IEMA IOT" ,"IEMA SENTINELSENSE", "WARP"};
const char* password[3] = {"Iot@iema_23", "IEMA6012", "ankg2279"};

const char* topic = "IEMA/WST/";
char device_id[17] = "";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedDate;
String ota_time1_1 = "19:00:00";
String ota_time1_2 = "19:00:01";
String ota_time1_3 = "19:00:02";
String ota_time1_4 = "19:00:03";
String ota_time1_5 = "19:00:04";

String ota_time2_1 = "03:00:00";
String ota_time2_2 = "03:00:01";
String ota_time2_3 = "03:00:02";
String ota_time2_4 = "03:00:03";
String ota_time2_5 = "03:00:04";

String ota_time3_1 = "11:00:00";
String ota_time3_2 = "11:00:01";
String ota_time3_3 = "11:00:02";
String ota_time3_4 = "11:00:03";
String ota_time3_5 = "11:00:04";

const char *mqtt_username = "iema_iot";
const char *mqtt_password = "12345";
// Add your MQTT Broker IP address, example:
// const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_server = "broker.emqx.io";
// const char* mqtt_server = "192.168.50.9";

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
int air_quality     = 0.0;
int windspeed       = 0.0;
String dayStamp;
String timeStamp;
uint16_t PM2_5 = 0;
uint16_t PM1_0 = 0;
uint16_t PM10 = 0;
bool sensorReadSuccessful = false;

DHT dht(DHTPIN, DHTTYPE);

String ranchar = "serverIndex";
/* Style */
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

/* Login page */
String loginIndex = 
"<form name=loginForm>"
"<h1>User Login</h1>"
"<input name=userid placeholder='User ID'> "
"<input name=pwd placeholder=Password type=Password> "
"<input type=submit onclick=check(this.form) class=btn value=Login></form>"
"<script>"
"function check(form) {"
"if(form.userid.value=='iema_admin' && form.pwd.value=='iema@8127')"
"{window.open('/" + ranchar + "')}"
"else"
"{alert('Error Password or Username')}"
"}"
"</script>" + style;
 
/* Server Index Page */
String serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>" + style;

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
  
  if (!bme.begin() || !particle.begin()) {
    Serial.println("Could not find a valid sensor, check wiring!");
    delay(5000);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  particle.gainVersion();

  /*use mdns for host name resolution*/
  
  delay(1000);
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
  if (!MDNS.begin(host)) { //http://iema_iot.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/" + ranchar, HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
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
  while (WiFi.status() != WL_CONNECTED)
    setup_wifi();
    server.handleClient();
    updateTimeGMT();
    delay(100);
    if(    timeStamp == ota_time1_1 || timeStamp == ota_time1_2 || timeStamp == ota_time1_3 || timeStamp == ota_time1_4 || timeStamp == ota_time1_5
        || timeStamp == ota_time2_1 || timeStamp == ota_time2_2 || timeStamp == ota_time2_3 || timeStamp == ota_time2_4 || timeStamp == ota_time2_5
        || timeStamp == ota_time3_1 || timeStamp == ota_time3_2 || timeStamp == ota_time3_3 || timeStamp == ota_time3_4 || timeStamp == ota_time3_5){
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

              PM2_5 = particle.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
              PM1_0 = particle.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
              PM10 = particle.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
              if(PM10 == 51519 && PM2_5 == 51519 && PM1_0 == 51519){
                    PM10 = 0;
                    PM1_0 = 0;
                    PM2_5 = 0;
              }
          
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
              JSONencoder["pm_2_5"]         = PM2_5;
              JSONencoder["pm_1_0"]         = PM1_0;
              JSONencoder["pm_10"]          = PM10;
              JSONencoder["wind_speed"]     = windspeed;
              JSONencoder["timestamp"]      = formattedDate;

              
           
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