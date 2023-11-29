#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// #include <Adafruit_GFX.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <BH1750.h>
#include <SimpleTimer.h>
#include "time.h"
#include "MHZ19.h"

// #include <LiquidCrystal_I2C.h>

#include <ItemInput.h>
#include <LcdMenu.h>
#include <ItemToggle.h>
#include <ItemProgress.h>
#include <ItemSubMenu.h>
#include <ItemCommand.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

SimpleTimer timer;

#define LCD_ROWS 4
#define LCD_COLS 20

const int buttonUpPin = 16;   // Button for moving up in the menu
const int buttonDownPin = 17; // Button for moving down in the menu
const int buttonMenuPin = 18; // Button for entering the menu
const int buttonOkPin = 15;   // Button for confirming a menu selection
const int buttonLeftPin = 5;
const int buttonRightPin = 4;

void inputCallback(char *value);



// Declare sensor variables
Adafruit_BME280 bme;
MHZ19 myMHZ19;
HardwareSerial mySerial(0); // On ESP32, we use HardwareSerial

LiquidCrystal_I2C lcd(0x27, 20, 4);

BH1750 lightMeter;

bool taskCompleted = false;
// unsigned long timestamp;
FirebaseJson json;

int temperature;
int humidity;
int CO2;
int lux;


// WiFi and Firebase credentials
#define WIFI_SSID "KSP"
#define WIFI_PASSWORD "9550421866"
// #define WIFI_SSID "Airtel_Joyenggservices"
// #define WIFI_PASSWORD "Joy@2022"
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
// TEMP SET POINTS
int tempSetPointOff;
int tempSetPointOn;

// HUM SET POINTS
int humdSetPointOff;
int humdSetPointOn;

// DEHUM SET POINTS
int dehumdSetPointOff;
int dehumdSetPointOn;

// Variables to save database paths
String listenerPath = "board1/outputs/digital/";

#define MAX_WIFI_RECONNECT_ATTEMPTS 5
#define WIFI_CONNECT_TIMEOUT 30 // seconds

// Declare outputs
const int Relay1 = 14; // mm
const int Relay2 = 27; // mm
const int Relay3 = 26; // mm
const int Relay4 = 25;
const int Relay5 = 13;
const int Relay6 = 4;
const int Relay7 = 23;
const int Relay8 = 19;



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
    dehumdSetPointOff = data.intData();
    Serial.print("deHumd Set Point Off: ");
    Serial.println(dehumdSetPointOff);
  }
  else if (streamPath.endsWith("/dehumd_set_point_on"))
  {
    dehumdSetPointOn = data.intData();
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

  delay(5000); // Add delay after JSON changes block
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
  }
}

void displaySensorValues()
{
  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SH110X_WHITE);
  // display.setCursor(0, 0);
  // display.print("Temp: " + String(temperature) + "C");
  // display.setCursor(0, 10);
  // display.print("Humidity: " + String(humidity) + "%");
  // display.setCursor(0, 20);
  // display.print("CO2: " + String(CO2));
  // display.setCursor(0, 30);
  // display.print("Lux: " + String(lux));
  // display.display();
}

void uploadData()
{
  taskCompleted = false;
  // Serial.println(" False");
}

void getRealTimeSensorsData()
{
  delay(3000);
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  CO2 = myMHZ19.getCO2();
  lux = lightMeter.readLightLevel();
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
    // displaySensorValues();

    Serial.print("Humidity: ");
    Serial.print(humidity);

    Serial.println("%");

    Serial.print("CO2: ");
    Serial.println(CO2);

    Serial.print("Lux: ");
    Serial.println(lux);

    FirebaseJson content;
    // timestamp = getTime();

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
      Serial.println("Failed to obtain time");
      return;
    }

    // Print timestamp
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d-%H-%M", &timeinfo);
    Serial.print("Timestamp: ");
    Serial.println(timestamp);

    // String formattedTimestamp = Firebase.timestamp(timestamp);

    String documentPath = "device1/" + String(timestamp);

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
  Serial.begin(9600);

  lcd.init(); // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Hello, world!");

  initWiFi();

  configTime(5.5 * 3600, 0, "pool.ntp.org"); // India has a UTC offset of 5 hours and 30 minutes

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

  // TEMP
  //  Retrieve initial values from Firebase
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_off"))
  //   tempSetPointOff = fbdo.intData();
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_on"))
  //   tempSetPointOn = fbdo.intData();

  // // HUM
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_off"))
  //   humdSetPointOff = fbdo.intData();
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_on"))
  //   humdSetPointOn = fbdo.intData();

  // // DEHUM
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "dehumd_set_point_off"))
  //   dehumdSetPointOff = fbdo.intData();
  // if (Firebase.RTDB.get(&fbdo, listenerPath + "dehumd_set_point_on"))
  //   dehumdSetPointOn = fbdo.intData();

  // Serial.print("Dehumd Set Point On: ");
  // Serial.println(dehumdSetPointOn);
  // Serial.print("Dehumd Set Point Off: ");
  // Serial.println(dehumdSetPointOff);
  // Serial.print("Humd Set Point On: ");
  // Serial.println(humdSetPointOn);
  // Serial.print("Humd Set Point Off: ");
  // Serial.println(humdSetPointOff);
  // Serial.print("Temp Set Point On: ");
  // Serial.println(tempSetPointOn);
  // Serial.print("Temp Set Point Off: ");
  // Serial.println(tempSetPointOff);

  if (!Firebase.RTDB.beginStream(&stream, listenerPath.c_str()))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());

  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);
  delay(2000);

  Wire.begin();
  mySerial.begin(9600);
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration();

  lightMeter.begin();

  bool status = bme.begin(0x76);
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  // timer.setInterval(3000, uploadData);
  // timer.setInterval(600000, SensorDataUpload);
  timer.setInterval(60000, SensorDataUpload);
  // timer.setInterval(3000, getRealTimeSensorsData);
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
  getRealTimeSensorsData();
  if (!isnan(temperature))
  {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    displaySensorValues();

    if (temperature >= tempSetPointOn) // 24
    {
      delay(3000);
      digitalWrite(Relay1, LOW); // Turn on relay
      Serial.println("Relay1 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay1 turned ON (C)");
      // Add LCD update code if needed
    }
    else if (temperature <= tempSetPointOff) // 22
    {
      delay(3000);
      digitalWrite(Relay1, HIGH); // Turn off relay
      Serial.println("Relay1 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay1 turned OFF (C)");
      // Add LCD update code if needed
    }
  }

  // Dehumidity  ---- heater

  if (!isnan(humidity))
  {
    Serial.print("humidity: ");
    Serial.println(humidity);

    // displaySensorValues();

    // Dehumidification (Relay2)
    if (humidity >= dehumdSetPointOn) // 66 > 98
    {
      // delay(3000);
      digitalWrite(Relay2, LOW); // Turn on relay
      Serial.print("dehumdidity Set Point ON: ");
      Serial.println(dehumdSetPointOn);
      Serial.println("Relay2 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("R2 turned ON (DH)");
      // Add LCD update code if needed
    }
    else if (humidity <= dehumdSetPointOff) // 66 < 95
    {
      // delay(3000);
      digitalWrite(Relay2, HIGH); // Turn off relay
      Serial.print("dehumdidity Set Point OFF: ");
      Serial.println(dehumdSetPointOff);
      Serial.println("Relay2 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("R2 turned oFF (DH)");
    }

    // humidity (Relay3)
    if (humidity < humdSetPointOn) // 24
    {

      // Serial.println(humidity + "<" + humdSetPointOff);
      delay(3000);
      digitalWrite(Relay3, LOW); // Turn on relay
      Serial.print("humdidity Set Point ON: ");
      Serial.println(humdSetPointOn);
      Serial.println("Relay3 turned ON");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay3 turned ON (H)");
      // Add LCD update code if needed
    }
    else if (humidity > humdSetPointOff)
    {
      delay(3000);

      // Serial.println(String(humidity) + ">" + humdSetPointOff);
      digitalWrite(Relay3, HIGH); // Turn off relay
      Serial.print("humdidity Set Point OFF: ");
      Serial.println(humdSetPointOff);
      Serial.println("Relay3 turned OFF");
      // lcd.setCursor(0, 3);
      // lcd.print("Relay3 turned OFF (H)");
      // Add LCD update code if needed
    }
  }
}
