#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <BH1750.h>
#include <SimpleTimer.h>
#include "time.h"
#include "MHZ19.h"

// #include <LiquidCrystal_I2C.h>

#include <ESP32Encoder.h>
#include <ItemToggle.h>
#include <ItemSubMenu.h>
#include <ItemCommand.h>
#include <ItemInput.h>
#include <LcdMenu.h>
#include <OneButton.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

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

// gobal char Converter to the text

char tempOnValue[20];
char tempOffValue[20];
char dehumOnValue[20];
char dehumOffValue[20];
char humdOnValue[20];
char humdOffValue[20];

SimpleTimer timer;

// LCD Menu //
#define LCD_ROWS 4
#define LCD_COLS 20

// Encoder Pins
#define ROTARY_ENCODER_CLK 16 // Replace with your CLK pin 4
#define ROTARY_ENCODER_DT 5   // Replace with your DT pin
#define ROTARY_ENCODER_SW 17  // Replace with your SW (button) pin 19

// PUSH BUTTONS DEFINE

const int buttonMenuPin = 18; // Button for entering the menu
const int buttonOkPin = 15;   // Button for confirming a menu selection

// ENCODER DEFINE
ESP32Encoder encoder;

// ONEBUTTON AND ENCODER DEFINE
OneButton button(ROTARY_ENCODER_SW, true);

// DOUBLE CLICK AND SINGLE CLICK
unsigned long lastButtonClickTime = 0;
const unsigned long doubleClickTimeThreshold = 500; // milliseconds

// SET POINTS VALUE
#define CHARSET_SIZE 10
// Create your charset
char charset[] = {
    '0',
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
};

LcdMenu menu(LCD_ROWS, LCD_COLS);
void inputCallback(char *value);
void toggleBacklight(uint16_t isOn);
uint8_t charsetPosition;
void hideMenu();

volatile bool dataChanged = false;

volatile bool sensorsData = true;

void handleButtonClick()
{
  // menu.enter(); // Perform "Enter" action on button push
  unsigned long currentTime = millis();
  if (currentTime - lastButtonClickTime <= doubleClickTimeThreshold)
  {
    // Perform "Double-Click" action
    menu.back(); // Go back in the menu
    delay(200);
  }
  else
  {
    // Perform "Single-Click" action//

    menu.type(charset[charsetPosition]);

    menu.enter(); // Enter the selected menu item
    delay(200);
  }

  lastButtonClickTime = currentTime;
}

void inputTempSetPointONCallback(char *value);
void inputTempSetPointOFFCallback(char *value);

void inputDehubSetPointONCallback(char *value);
void inputDehubSetPointOFFCallback(char *value);

void inputHumSetPointONCallback(char *value);
void inputHumSetPointOFFCallback(char *value);

extern MenuItem *tempMenu[];
extern MenuItem *dehumMenu[];
extern MenuItem *humMenu[];

MAIN_MENU(
    // ITEM_INPUT("Temp SP_ON", inputTempSetPointONCallback),
    ITEM_SUBMENU("Temperature", tempMenu),
    ITEM_SUBMENU("Dehumidity", dehumMenu),
    ITEM_SUBMENU("Humidity", humMenu),
    ITEM_COMMAND("Hide Menu", hideMenu),
    ITEM_TOGGLE("Backlight", toggleBacklight),
    ITEM_BASIC("Connect to WiFi"));

SUB_MENU(tempMenu, mainMenu,
         ITEM_INPUT("Temp SP_ON", inputTempSetPointONCallback),
         ITEM_INPUT("Temp SP_OFF", inputTempSetPointOFFCallback));

SUB_MENU(dehumMenu, mainMenu,
         ITEM_INPUT("Dehum SP_ON", inputDehubSetPointONCallback),
         ITEM_INPUT("Dehum SP_OFF", inputDehubSetPointOFFCallback));

SUB_MENU(humMenu, mainMenu,
         ITEM_INPUT("Humi SP_ON", inputHumSetPointONCallback),
         ITEM_INPUT("Humi SP_OFF", inputHumSetPointOFFCallback));
// LCD MENU

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

// Variables to save database paths
String listenerPath = "board1/outputs/digital/";

String parentPath;

// WIFI Reconnect Method
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

  dataChanged = true;
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
    menu.lcd->clear();
    menu.lcd->setCursor(0, 0);
    menu.lcd->print("Connecting to WiFi...");
  }
  Serial.print("Connected to");
  Serial.println(WIFI_SSID);
  menu.lcd->clear();
  menu.lcd->setCursor(0, 0);
  menu.lcd->print("Connected to  ");
  menu.lcd->println(WIFI_SSID);

  delay(2000);
}

void displaySensorValues()
{

  menu.lcd->clear();
  menu.lcd->setCursor(0, 0);
  menu.lcd->print("Temperature: ");
  menu.lcd->print(temperature);
  menu.lcd->print(" C");

  menu.lcd->setCursor(0, 1);
  menu.lcd->print("Humidity: ");
  menu.lcd->print(humidity);
  menu.lcd->print(" %");

  menu.lcd->setCursor(0, 2);
  menu.lcd->print("CO2: ");
  menu.lcd->print(CO2);
  menu.lcd->print(" ppm");

  menu.lcd->setCursor(0, 3);
  menu.lcd->print("Lux: ");
  menu.lcd->print(lux);
  menu.lcd->print(" lux");

  delay(3000);
}

void uploadData()
{
  taskCompleted = false;
  // Serial.println(" False");
}

// String parentPath;
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

    String documentPath = "device10/" + String(timestamp);

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

// void taskMenu(void *pvParameters)
// {
//   Serial.print("Task1 running on core ");
//   Serial.println(xPortGetCoreID());
//   for (;;)
//   {
//     if (sensorsData)
//     {
//       displaySensorValues();
//     }
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//   }
// }

void taskEncoder(void *pvParameters)
{

  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for (;;)
  {
    int encoderValue = encoder.getCount();
    if (encoderValue > 0)
    {
      if (menu.isInEditMode())
      {
        charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
        menu.drawChar(charset[charsetPosition]);
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }
      menu.down();
    }
    else if (encoderValue < 0)
    {
      if (menu.isInEditMode())
      {
        charsetPosition = constrain(charsetPosition - 1, 0, CHARSET_SIZE);
        menu.drawChar(charset[charsetPosition]);
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }
      menu.up();
    }
    encoder.clearCount();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void taskButton(void *pvParameters)
{
  for (;;)
  {
    button.tick();

    if (digitalRead(buttonMenuPin) == LOW)
    {
      Serial.println("Button Menu back Pressed");
      menu.back();
    }
    else if (digitalRead(buttonOkPin) == LOW)
    {
      Serial.println("Button OK Pressed");

      if (menu.isInEditMode())
      {
        menu.backspace();
        Serial.println("BackSpace");
        vTaskDelay(200 / portTICK_PERIOD_MS);
      }

      menu.show();
      sensorsData = false;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Reduce the delay
  }
}

void getSetPointsData()
{

  // TEMP
  // Retrieve initial values from Firebase
  if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_off"))
    tempSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "temp_set_point_on"))
    tempSetPointOn = fbdo.intData();

  // HUM
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_off"))
    humdSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "humd_set_point_on"))
    humdSetPointOn = fbdo.intData();

  // DEHUM
  if (Firebase.RTDB.get(&fbdo, listenerPath + "dehumd_set_point_off"))
    dehumdSetPointOff = fbdo.intData();
  if (Firebase.RTDB.get(&fbdo, listenerPath + "dehumd_set_point_on"))
    dehumdSetPointOn = fbdo.intData();

  Serial.print("Dehumd Set Point On: ");
  Serial.println(dehumdSetPointOn);
  Serial.print("Dehumd Set Point Off: ");
  Serial.println(dehumdSetPointOff);
  Serial.print("Humd Set Point On: ");
  Serial.println(humdSetPointOn);
  Serial.print("Humd Set Point Off: ");
  Serial.println(humdSetPointOff);
  Serial.print("Temp Set Point On: ");
  Serial.println(tempSetPointOn);
  Serial.print("Temp Set Point Off: ");
  Serial.println(tempSetPointOff);
}

void lcdSetPointDataPrint()
{
  // Set numeric values for temperature menu
  sprintf(tempOnValue, "%d", tempSetPointOn);
  sprintf(tempOffValue, "%d", tempSetPointOff);
  tempMenu[1]->setValue(tempOnValue);
  tempMenu[2]->setValue(tempOffValue);

  // Set numeric values for dehumidifier menu
  sprintf(dehumOnValue, "%d", dehumdSetPointOn);
  sprintf(dehumOffValue, "%d", dehumdSetPointOff);
  dehumMenu[1]->setValue(dehumOnValue);
  dehumMenu[2]->setValue(dehumOffValue);

  // Set numeric values for humidity menu
  sprintf(humdOnValue, "%d", humdSetPointOn);
  sprintf(humdOffValue, "%d", humdSetPointOff);
  humMenu[1]->setValue(humdOnValue);
  humMenu[2]->setValue(humdOffValue);

  menu.update();
}

void setup()
{
  Serial.begin(9600);

  // LCD Menu
  menu.setupLcdWithMenu(0x27, mainMenu);
  menu.lcd->clear();
  menu.lcd->setCursor(0, 0);
  menu.lcd->print("Getting Started");
  initWiFi();

  menu.hide();
  menu.lcd->clear();
  menu.lcd->setCursor(0, 0);
  menu.lcd->print("initializing \n Sensors");

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

  // ONE BUTTON
  button.attachClick(handleButtonClick);

  // Attach the encoder
  encoder.attachHalfQuad(ROTARY_ENCODER_CLK, ROTARY_ENCODER_DT);

  pinMode(buttonMenuPin, INPUT_PULLUP);
  pinMode(buttonOkPin, INPUT_PULLUP);

  charsetPosition = 0;

  // Firebase DataChanger Method
  if (!Firebase.RTDB.beginStream(&stream, listenerPath.c_str()))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());

  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);

  // delay(2000);
  getSetPointsData();
  lcdSetPointDataPrint();
  // Sensors Getting Started
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

  if (sensorsData)
  {

    displaySensorValues();
  }

  // timer.setInterval(3000, uploadData);
  // timer.setInterval(600000, SensorDataUpload);
  timer.setInterval(60000, SensorDataUpload);
  // timer.setInterval(3000, getRealTimeSensorsData);

  // xTaskCreatePinnedToCore(
  //     taskMenu,   /* Task function. */
  //     "TaskMenu", /* name of task. */
  //     10000,      /* Stack size of task */
  //     NULL,       /* parameter of the task */
  //     1,          /* priority of the task */
  //     NULL,       /* Task handle to keep track of created task */
  //     0);         /* pin task to core 0 */

  xTaskCreatePinnedToCore(
      taskEncoder,   /* Task function. */
      "TaskEncoder", /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      NULL,          /* Task handle to keep track of created task */
      0);            /* pin task to core 0 */

  xTaskCreatePinnedToCore(
      taskButton,   /* Task function. */
      "TaskButton", /* name of task. */
      10000,        /* Stack size of task */
      NULL,         /* parameter of the task */
      1,            /* priority of the task */
      NULL,         /* Task handle to keep track of created task */
      0);           /* pin task to core 0 */
}

void loop()
{

  // Wifi Reconnecting Methos
  if (WiFi.status() != WL_CONNECTED)
  {
    reconnectWiFi();
  }
  // Timer Run For Sensors Data Uploading
  timer.run();

  Serial.print("loop() running on core ");
  Serial.println(xPortGetCoreID());

  // int tempSetPointOn =50;

  // LCD Menu For Encoder
  // menuEncoderKsp();

  // Getting Firebase Token (Upload data/Get Data)
  if (Firebase.isTokenExpired())
  {
    Firebase.refreshToken(&configF);
    Serial.println("Refresh token");
  }

  if (dataChanged)
  {
    dataChanged = false;
    // When stream data is available, do anything here...
    getSetPointsData();
    lcdSetPointDataPrint();
  }

  if (sensorsData)
  {
    displaySensorValues();
  }

  // Get Sensors realtime Data
  getRealTimeSensorsData();

  // SET POINTS OF TEMPERATURE
  if (!isnan(temperature))
  {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    // displaySensorValues();

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

  //  Dehumidity  ---- heater
  // SET POINTS OF TEMPERATURE
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

  button.tick();
}

// LCD Menu

void inputCallback(char *value)
{
  // Serial.print(F("# "));
  Serial.println(value);
}

void updateSetPoint(const char *setPointType, const char *setPointName, char *value)
{
  // Check if the value is not null and not zero
  if (value != nullptr && atof(value) != 0)
  {
    if (Firebase.ready())
    {
      FirebaseJson json;
      parentPath = setPointType;
      json.set(setPointName, atof(value));

      // Uncomment the following line if you want to print the JSON data
      // Serial.printf("JSON Data: %s\n", json.toStdString().c_str());

      Serial.printf("Update node... %s\n", Firebase.RTDB.updateNode(&fbdo, listenerPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
      menu.lcd->clear();
      menu.lcd->setCursor(0, 0);
      menu.lcd->println("Saved Value");

      delay(5000);
      menu.show();
    }
    else
    {
      Serial.println("Firebase not ready. Node not updated.");
      menu.lcd->clear();

      menu.lcd->setCursor(0, 0);
      menu.lcd->println("Firebase not");
      menu.lcd->setCursor(0, 1);
      menu.lcd->println("ready. Node");
      menu.lcd->setCursor(0, 2);
      menu.lcd->println("not updated.");

      delay(5000);
      // menu.lcd->setCursor(0, 0);
      // menu.lcd->println("");
      menu.show();
    }
  }
  else
  {
    Serial.println("Value is null or zero. Node not updated.");
    menu.lcd->clear();
    menu.lcd->setCursor(0, 0);
    menu.lcd->println("Value is null or ");
    menu.lcd->setCursor(0, 1);
    menu.lcd->println("zero. Node not ");
    menu.lcd->setCursor(0, 2);
    menu.lcd->println("updated. ");

    delay(5000);
    // menu.lcd->setCursor(3, 0);
    // menu.lcd->println("");
    menu.show();
  }

  lcdSetPointDataPrint();
}

void inputTempSetPointONCallback(char *value)
{
  updateSetPoint("temp_set_point_on", "temp_set_point_on", value);
}

void inputTempSetPointOFFCallback(char *value)
{
  updateSetPoint("temp_set_point_off", "temp_set_point_off", value);
}

void inputDehubSetPointONCallback(char *value)
{
  updateSetPoint("dehumd_set_point_on", "dehumd_set_point_on", value);
}

void inputDehubSetPointOFFCallback(char *value)
{
  updateSetPoint("dehumd_set_point_off", "dehumd_set_point_off", value);
}

void inputHumSetPointONCallback(char *value)
{
  updateSetPoint("humd_set_point_on", "humd_set_point_on", value);
}

void inputHumSetPointOFFCallback(char *value)
{
  // Check if the value is non-zero and not null

  updateSetPoint("humd_set_point_off", "humd_set_point_off", value);
}

// BackLight ON/OFF
void toggleBacklight(uint16_t isOn)
{
  menu.setBacklight(isOn);
}

// Menu HideMenu

void hideMenu()
{
  // Perform actions to hide the menu

  Serial.println("Menu Hide");
  menu.hide(); // Clear the LCD screen or perform other actions as needed
  // delay(100);
  sensorsData = true;
}