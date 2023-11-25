#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
// #include <DallasTemperature.h>
#include <BH1750.h>
// #include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include "time.h"
#include "MHZ19.h"
#include <Adafruit_SH110X.h>

// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <WebSerial.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

SimpleTimer timer;

// AsyncWebServer server(80);

// void recvMsg(uint8_t *data, size_t len)
// {
//   WebSerial.println("Received Data...");
//   // String d = "";
//   // for (int i = 0; i < len; i++)
//   // {
//   //   d += char(data[i]);
//   // }
//   // WebSerial.println(d);
//   // if (d == "ON")
//   // {
//   //   digitalWrite(LED, HIGH);
//   // }
//   // if (d == "OFF")
//   // {
//   //   digitalWrite(LED, LOW);
//   // }
// }
// Declare sensor variables
Adafruit_BME280 bme;
MHZ19 myMHZ19;
HardwareSerial mySerial(0); // On ESP32, we use HardwareSerial

// LiquidCrystal_I2C lcd(0x27, 20, 4);
// OneWire oneWire(4); // Dallas Temperature sensor on pin 4
// DallasTemperature sensors(&oneWire);
BH1750 lightMeter;

bool taskCompleted = false;
unsigned long timestamp;
FirebaseJson json;

int temperature;
int humidity;
int CO2;
int lux;

#define i2c_Address 0x3c // initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    //   QT-PY / XIAO

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Adafruit_SH110X display(128, 64, &Wire); // Assuming you're using SH110X OLED

// WiFi and Firebase credentials
// #define WIFI_SSID "KSP"
// #define WIFI_PASSWORD "9550421866"
#define WIFI_SSID "Airtel_Joyenggservices"
#define WIFI_PASSWORD "Joy@2022"
#define API_KEY "AIzaSyCiIG-sTPSX06NeqO1oKY45g6z1xxT56Lw"
#define FIREBASE_PROJECT_ID "ksp-iot"
#define USER_EMAIL "device@admin.com"
#define USER_PASSWORD "123456789"
#define DATABASE_URL "ksp-iot-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig configF;

// Global variables
int tempSetPointOff;
int humdSetPointOff;
int humdSetPointOn;
int tempSetPointOn;
int dehumdSetPointOff;
int dehumdSetPointOn;

// Variables to save database paths
String listenerPath = "board1/outputs/digital/";

// Declare outputs
const int Relay1 = 14; // mm
const int Relay2 = 27; // mm
const int Relay3 = 26; // mm
const int Relay4 = 25;
const int Relay5 = 13;
const int Relay6 = 4;
const int Relay7 = 23;
const int Relay8 = 19;

#define MAX_WIFI_RECONNECT_ATTEMPTS 5
#define WIFI_CONNECT_TIMEOUT 30 // seconds

// Callback function that runs on database changes
void streamCallback(FirebaseStream data)
{
  Serial.printf("stream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); // see addons/RTDBHelper.h
  Serial.println();

  // Get the path that triggered the function
  String streamPath = String(data.dataPath());
  Serial.print(streamPath);

  // Get the path that triggered the function
  // String streamPath = String(data.dataPath());

  // Handle paths for temperature and humidity set points
  if (streamPath.endsWith("/temp_set_point_off"))
  {
    tempSetPointOff = data.intData();
    Serial.print("Temp Set Point Off: ");
    Serial.println(tempSetPointOff);
  }
  else if (streamPath.endsWith("/humd_set_point_off"))
  {
    humdSetPointOff = data.intData();
    Serial.print("Humd Set Point Off: ");
    Serial.println(humdSetPointOff);
  }
  else if (streamPath.endsWith("/humd_set_point_on"))
  {
    humdSetPointOn = data.intData();
    Serial.print("Humd Set Point On: ");
    Serial.println(humdSetPointOn);
  }
  else if (streamPath.endsWith("/temp_set_point_on"))
  {
    tempSetPointOn = data.intData();
    Serial.print("Temp Set Point On: ");
    Serial.println(tempSetPointOn);
  }
  else if (streamPath.endsWith("/dehumd_set_point_off"))
  {
    humdSetPointOff = data.intData();
    Serial.print("deHumd Set Point Off: ");
    Serial.println(dehumdSetPointOff);
  }
  else if (streamPath.endsWith("/dehumd_set_point_on"))
  {
    humdSetPointOn = data.intData();
    Serial.print("deHumd Set Point On: ");
    Serial.println(dehumdSetPointOn);
  }

  // Handle GPIO changes
  // if the data returned is an integer, there was a change on the GPIO state on the following path /{gpio_number}
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_integer)
  {
    String gpio = streamPath.substring(1);
    int state = data.intData();
    Serial.print("GPIO: ");
    Serial.println(gpio);
    Serial.print("STATE: ");
    Serial.println(state);
    digitalWrite(gpio.toInt(), state);
  }

  // Handle JSON changes
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json)
  {
    FirebaseJson json = data.to<FirebaseJson>();
    size_t count = json.iteratorBegin();
    Serial.println("\n---------");
    for (size_t i = 0; i < count; i++)
    {
      FirebaseJson::IteratorValue value = json.valueAt(i);
      int gpio = value.key.toInt();
      int state = value.value.toInt();
      Serial.print("STATE: ");
      Serial.println(state);
      Serial.print("GPIO:");
      Serial.println(gpio);
      digitalWrite(gpio, state);
      Serial.printf("Name: %s, Value: %s, Type: %s\n", value.key.c_str(), value.value.c_str(), value.type == FirebaseJson::JSON_OBJECT ? "object" : "array");
    }
    Serial.println();
    json.iteratorEnd();
  }

  Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
}

void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

void initWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Connecting to WiFi...");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Connected to");
  display.setCursor(0, 10);
  display.print(WIFI_SSID);
}

unsigned long getTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return 0;
  }
  time(&now);
  return now;
}

void displaySensorValues()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Temp: " + String(temperature) + "C");
  display.setCursor(0, 10);
  display.print("Humidity: " + String(humidity) + "%");
  display.setCursor(0, 20);
  display.print("CO2: " + String(CO2));
  display.setCursor(0, 30);
  display.print("Lux: " + String(lux));
  display.display();
}

void uploadData()
{
  taskCompleted = false;
  Serial.println(" False");
}

void SensorDataUpload()
{
  if (Firebase.ready())
  {
    Serial.print("Getting Sensor Data");

    // Fetch sensor data
    // sensors.requestTemperatures();
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    CO2 = myMHZ19.getCO2();
    lux = lightMeter.readLightLevel();
    displaySensorValues();

    Serial.print("Humidity: ");
    Serial.print(humidity);

    Serial.println("%");

    Serial.print("CO2: ");
    Serial.println(CO2);

    Serial.print("Lux: ");
    Serial.println(lux);

    FirebaseJson content;
    timestamp = getTime();

    // String formattedTimestamp = Firebase.timestamp(timestamp);

    String documentPath = "devices/device1/" + String(timestamp);

    // content.set("fields/temp/doubleValue", sensors.getTempCByIndex(0));

    content.set("fields/temp/doubleValue", temperature);
    content.set("fields/humidity/doubleValue", humidity);
    content.set("fields/CO2/integerValue", CO2);
    content.set("fields/lux/doubleValue", lux);
    content.set("fields/myTimestamp/stringValue", String(timestamp)); // RFC3339 UTC "Zulu" format

    Serial.print("Create a document... ");
    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw()))
      Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    else
      Serial.println(fbdo.errorReason());
  }
}

void reconnectWiFi()
{
  Serial.print("Attempting to reconnect to WiFi");
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < MAX_WIFI_RECONNECT_ATTEMPTS)
  {
    Serial.print(".");

    // Attempt to reconnect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for the connection to be established
    int attemptTimeout = 0;
    while (WiFi.status() != WL_CONNECTED && attemptTimeout < WIFI_CONNECT_TIMEOUT)
    {
      delay(1000);
      Serial.print(".");
      attemptTimeout++;
    }

    attempts++;
    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nReconnected to WiFi. IP address: " + WiFi.localIP().toString());
  }
  else
  {
    Serial.println("\nFailed to reconnect to WiFi. Restarting the board...");
    ESP.restart(); // Reset the board
  }
}

void setup()
{
  Serial.begin(115200);
  // display.begin(i2c_Address, true);
  display.begin(i2c_Address, true); // Address 0x3C default

  display.display();
  display.clearDisplay();
  initWiFi();
  configTime(0, 0, "pool.ntp.org");

  // Initialize Outputs
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(Relay5, OUTPUT);
  pinMode(Relay6, OUTPUT);
  pinMode(Relay7, OUTPUT);
  pinMode(Relay8, OUTPUT);

  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
  digitalWrite(Relay5, HIGH);
  digitalWrite(Relay6, HIGH);
  digitalWrite(Relay7, HIGH);
  digitalWrite(Relay8, HIGH);

  configF.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  configF.token_status_callback = tokenStatusCallback;
  configF.max_token_generation_retry = 5;

  configF.database_url = DATABASE_URL;

  Firebase.begin(&configF, &auth);
  Firebase.reconnectWiFi(true);

  // Retrieve initial values from Firebase
  if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_off"))
    tempSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_off"))
    humdSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_on"))
    humdSetPointOn = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_on"))
    tempSetPointOn = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_off"))
    dehumdSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_on"))
    dehumdSetPointOn = fbdo.intData();

  if (!Firebase.RTDB.beginStream(&stream, listenerPath.c_str()))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());

  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);
  delay(2000);

  mySerial.begin(9600);
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration();
  Wire.begin();
  lightMeter.begin();

  bool status = bme.begin(0x76);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  timer.setInterval(3000, uploadData);
  timer.setInterval(600000, SensorDataUpload);
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWiFi();
  }
  timer.run();

  if (Firebase.isTokenExpired())
  {
    Firebase.refreshToken(&configF);
    Serial.println("Refresh token");
  }

  // WebSerial.print("Humidity: ");
  // WebSerial.print(humidity);

  // WebSerial.println("%");

  // WebSerial.print("CO2: ");
  // WebSerial.println(CO2);

  // WebSerial.print("Lux: ");
  // WebSerial.println(lux);

  if (!isnan(temperature))
  {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    displaySensorValues();

    if (temperature > tempSetPointOn) // 24
    {
      delay(3000);
      digitalWrite(Relay1, LOW); // Turn on relay
      Serial.println("Relay1 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned ON (C)");
      // Add LCD update code if needed
    }
    else if (temperature < tempSetPointOff) // 22
    {
      delay(3000);
      digitalWrite(Relay1, HIGH); // Turn off relay
      Serial.println("Relay1 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned OFF (C)");
      // Add LCD update code if needed
    }
  }

  if (!isnan(humidity))
  {
    Serial.print("humidity: ");
    Serial.println(humidity);

    displaySensorValues();

    if (humidity > dehumdSetPointOn) // 24
    {
      delay(3000);
      digitalWrite(Relay2, LOW); // Turn on relay
      Serial.println("Relay2 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned ON (H)");
      // Add LCD update code if needed
    }
    else if (humidity < dehumdSetPointOff)
    {
      delay(3000);
      digitalWrite(Relay2, HIGH); // Turn off relay
      Serial.println("Relay2 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned OFF (H)");
      // Add LCD update code if needed
    }
  }

  if (!isnan(humidity))
  {
    Serial.print("humidity: ");
    Serial.println(humidity);

    displaySensorValues();

    if (humidity > humdSetPointOn) // 24
    {
      delay(3000);
      digitalWrite(Relay3, LOW); // Turn on relay
      Serial.println("Relay3 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned ON (H)");
      // Add LCD update code if needed
    }
    else if (humidity < humdSetPointOff)
    {
      delay(3000);
      digitalWrite(Relay3, HIGH); // Turn off relay
      Serial.println("Relay3 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay turned OFF (H)");
      // Add LCD update code if needed
    }
  }
}
