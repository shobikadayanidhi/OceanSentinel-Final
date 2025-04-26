#include "DHT.h"
#include "WiFi.h"
#include "EmsolIoTx.h"
#include "SoftwareSerial.h"

// include the library code:
#include <LiquidCrystal.h>

//   The circuit:
//  * LCD RS pin to digital pin 16
//  * LCD Enable pin to digital pin 17
//  * LCD D4 pin to digital pin 5
//  * LCD D5 pin to digital pin 18
//  * LCD D6 pin to digital pin 19
//  * LCD D7 pin to digital pin 21
//  * LCD R/W pin to ground
//  * LCD VSS pin to ground
//  * LCD VCC pin to 5V

// Define ESP32 pins for LCD
const int rs = 16, en = 17, d4 = 5, d5 = 18, d6 = 19, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define BUTTON_PIN 15
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};
Button ButtonIntr = { BUTTON_PIN, 0, false };
//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;
unsigned long last_button_time = 0;

uint8_t io_control[] = { 26, 27 };  // LED PIN drive from Emsol Cloud
const int LED_PWM_PIN = 23;         // PWM pin connected to the LED controlled from Emsol IoTx
#define BUZZER_PIN 22
#define LDR_PIN 34     // LDR connected to GPIO 34 (ADC1 Channel 6)
#define DHTPIN 4       // Define the pin where the data wire is connected
#define DHTTYPE DHT11  // Define the DHT type, either DHT11 or DHT22
DHT dht(DHTPIN, DHTTYPE);

// Replace with your network credentials (STATION)
char ssid[] = "Pixel_2735";  // Must be 2.4GHz WiFi Band
char pass[] = "shifana2005";

// mosquitto_pub -d -q 1 -h 54.251.153.72 -p 1883 -t v1/devices/me/telemetry -u "btv4v9il6q2pvdd2xv9h" -m "{temperature:25}"
#define EMSOLIOTX_MQTT_SERVER "54.251.153.72"
#define EMSOLIOTX_MQTT_ACESSTOKEN "btv4v9il6q2pvdd2xv9h"
#define EMSOLIOTX_MQTT_PORT 1883

WiFiClient espClient;
EmsolIoTx emsoliotx(espClient);

int iotx_status = 0;
unsigned long previousMillis = 0;  // Stores the last time an action was taken
const long interval = 5000;        // Interval for periodic action (5 seconds)
unsigned int wifi_status = 0;
float previousHumidity = 0;
float previousTemperature = 0;

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))
// Array with IO that should be controlled from Emsol IoTx, one by one.
int led_delay = 0;  // Initial period of LED cycling.
bool subscribed = false;

int fadeAmount = 5;        // Amount to increase/decrease brightness by each cycle
const int freq = 5000;     // PWM frequency (5000 Hz)
const int ledChannel = 0;  // PWM channel (ESP32 has 16 channels)
const int resolution = 8;  // PWM resolution (8 bits, range: 0 - 255)

float brightness = 0;
float temperature = 34;
float humidity = 79;
float latitude = 12.971600;
float longitude = 77.594600;
int voilation = 0;

char displayLine1[17];  // 16 characters + null terminator
char displayLine2[17];  // 16 characters + null terminator

void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 500) {
    ButtonIntr.numberKeyPresses++;
    ButtonIntr.pressed = true;
    last_button_time = button_time;
  }
}

void printLatLong() {
  lcd.begin(16, 2);
  // // Format first line using snprintf
  // snprintf(displayLine1, sizeof(displayLine1), "T:%dC H:%d%% L:%d",
  //          (int)temperature, (int)humidity, (int)brightness);
  // lcd.setCursor(0, 0);
  // lcd.print(displayLine1);

  // // Print latitude and longitude on second line
  // lcd.setCursor(0, 1);
  // lcd.print(latitude, 4);
  // lcd.print(" ");
  // lcd.print(longitude, 4);
  // Print first line (Temperature, Humidity, Light)

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 0);
  lcd.print("C H:");
  lcd.print(humidity, 0);
  lcd.print("%");

  // Ensure brightness fits within 16 chars
  lcd.setCursor(12, 0);
  lcd.print("L:");
  lcd.print(brightness, 0);

  // Print second line (Latitude and Longitude)
  lcd.setCursor(0, 1);
  lcd.print(latitude, 4);
  lcd.print(" ");
  lcd.print(longitude, 4);
}

RPC_Response processDelayChange(const RPC_Data &data) {
  Serial.println("Received the set delay RPC method");
  led_delay = data;
  Serial.print("Set new delay: ");
  Serial.println(led_delay);
  int brightness = map(led_delay, 0, 100, 0, 255);
  // Set the LED brightness
  ledcWrite(ledChannel, brightness);
  return String(led_delay);
}

RPC_Response processGetDelay(const RPC_Data &data) {
  Serial.println("Received the get value method");
  return String(led_delay);
}

RPC_Response processSetGpioState(const RPC_Data &data) {
  Serial.println("Received the set GPIO RPC method");

  int pin = data["pin"];
  bool enabled = data["enabled"];

  Serial.print("Setting LED ");
  Serial.print(pin);
  Serial.print(" to state ");
  Serial.println(enabled);

  if (enabled == true)
    digitalWrite(pin, HIGH);
  else
    digitalWrite(pin, LOW);

  return String("{\"" + String(pin) + "\": " + String(enabled ? "true" : "false") + "}");
}

RPC_Response processGetGpioState(const RPC_Data &data) {
  Serial.println("Received the get GPIO RPC method");

  String respStr = "{";
  for (size_t i = 0; i < COUNT_OF(io_control); ++i) {
    int pin = io_control[i];
    Serial.print("Getting IO : ");
    Serial.print(pin);
    Serial.print(" state ");
    bool ioState = digitalRead(pin);
    Serial.println(ioState);
    respStr += String("\"" + String(pin) + "\": " + String(ioState ? "true" : "false") + ", ");
  }
  respStr = respStr.substring(0, respStr.length() - 2);
  respStr += "}";

  return respStr;
}

// RPC handlers
RPC_Callback callbacks[] = {
  { "setValue", processDelayChange },
  { "getValue", processGetDelay },
  { "setGpioStatus", processSetGpioState },
  { "getGpioStatus", processGetGpioState },
};

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  wifi_status = 1;
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid, pass);
}

void initWiFi() {
  WiFi.begin(ssid, pass);
  Serial.println("Connecting to WiFi Network.");
  int retry = 1;
  while (wifi_status != 1) {
    Serial.print('.');
    if (retry > 10) {
      break;
    }
    retry++;
    delay(1000);
  }

  if (retry > 10) {
    Serial.println("Failed to connect WiFi Network");
  } else {
    Serial.println("WiFi Connected.");
  }
}

void beep(int timeout_milliseconds) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(timeout_milliseconds);
  digitalWrite(BUZZER_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Emsol Learning Kit Test");
  lcd.begin(16, 2);
  printLatLong();


  pinMode(ButtonIntr.PIN, INPUT_PULLUP);
  attachInterrupt(ButtonIntr.PIN, isr, FALLING);
  pinMode(BUZZER_PIN, OUTPUT);

  for (size_t i = 0; i < COUNT_OF(io_control); ++i) {
    pinMode(io_control[i], OUTPUT);
  }

  // // Configure LED PWM functionalitites
  // ledcSetup(ledChannel, freq, resolution);
  // // Attach the LED pin to the PWM channel
  // ledcAttachPin(LED_PWM_PIN, ledChannel);

  // Delete old config
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  initWiFi();
}

void publishPayload(){
    if (iotx_status) {
    const size_t capacity = JSON_OBJECT_SIZE(20);
    DynamicJsonDocument doc(capacity);
    doc["temperature"] = String(temperature, 2);
    doc["humidity"] = String(humidity, 2);
    doc["brightness"] = String(brightness, 2);
    doc["voilation"] = String(voilation);
    doc["latitude"] = String(latitude, 4);
    doc["longitude"] = String(longitude, 4);

    String jsonString;
    serializeJson(doc, jsonString);
    Serial.print("Serialized JSON: ");
    Serial.println(jsonString);
    emsoliotx.sendTelemetryJson((char *)jsonString.c_str());
  }
}

void loop(void) {

  // Reading temperature or humidity takes about 250 milliseconds!
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();  // Reading temperature as Celsius
  int ldrValue = analogRead(LDR_PIN);   // Read the value from the LDR

  unsigned long currentMillis = millis();  // Get the current time
  // Check if 2 seconds have passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Save the last time the action was taken

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperature)) {
      humidity = 79;
      temperature = 28;
      Serial.println("Failed to read from DHT sensor!");
    } else {
      // Print the values to the serial monitor
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print(" %\t");
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
      previousHumidity = humidity;
      previousTemperature = temperature;
    }

    Serial.print("LDR Value (BRIGHTNESS): ");
    // Convert to brightness (0-100%)
    brightness = (ldrValue / 4095.0) * 100.0;

    // if (brightness < 20) {
    //   Serial.println(" => Dark");
    // } else if (brightness < 40) {
    //   beep(500);
    //   delay(500);
    //   beep(500);
    //   Serial.println(" => Dim");
    // } else if (brightness < 60) {
    //   beep(250);
    //   delay(100);
    //   beep(250);
    //   Serial.println(" => Light");
    // } else if (brightness < 80) {
    //   beep(100);
    //   delay(100);
    //   beep(100);
    //   Serial.println(" => Bright");
    // } else {
    //   Serial.println(" => Very bright");
    // }

    publishPayload();

    if (wifi_status) {
      // Reconnect to Emsol IoTx Cloud, if needed
      if (!emsoliotx.connected()) {
        // Connect to the Emsol IoTx Cloud
        Serial.print("Connecting MQTT to: ");
        Serial.print(EMSOLIOTX_MQTT_SERVER);
        Serial.print(" at port no. ");
        Serial.println(EMSOLIOTX_MQTT_PORT);
        Serial.print(" with access token ");
        Serial.println(EMSOLIOTX_MQTT_ACESSTOKEN);
        if (!emsoliotx.connect(EMSOLIOTX_MQTT_SERVER, EMSOLIOTX_MQTT_ACESSTOKEN, EMSOLIOTX_MQTT_PORT)) {
          iotx_status = 0;
          Serial.println("Failed to connect");
        } else {
          iotx_status = 1;
          Serial.println("Emsol IoTx Connected Successfully.");
        }
      }

      // Subscribe for RPC, if needed
      if (!subscribed) {
        Serial.println("Subscribing for RPC... ");

        // Perform a subscription. All consequent data processing will happen in
        // callbacks as denoted by callbacks[] array.
        if (!emsoliotx.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
          Serial.println("Failed to subscribe for RPC");
          return;
        }

        Serial.println("Subscribe done");
        subscribed = true;
      }
    }
  }


  if (ButtonIntr.pressed) {
    ButtonIntr.pressed = false;
    Serial.printf("Button has been pressed %u times\n", ButtonIntr.numberKeyPresses);
    if (ButtonIntr.numberKeyPresses == 5)
      ButtonIntr.numberKeyPresses = 1;

    switch (ButtonIntr.numberKeyPresses) {
      case 1:
        {
          latitude = 9.901216;
          longitude = 79.301268;
          brightness = 77;
          temperature = 31;
          humidity = 82;
          printLatLong();
          voilation = 0;
          publishPayload();
          digitalWrite(BUZZER_PIN, LOW);
        }
        break;
      case 2:
        {
          latitude = 9.695521;
          longitude = 79.444183;
          brightness = 70;
          // if (isnan(humidity) || isnan(temperature)) {
          temperature = 29;
          humidity = 78;
          // }
          printLatLong();
          voilation = 1;
          publishPayload();
          digitalWrite(BUZZER_PIN, HIGH);
        }
        break;
      case 3:
        {
          latitude = 9.251225;
          longitude = 79.444183;
          brightness = 65;
          // if (isnan(humidity) || isnan(temperature)) {
          temperature = 27;
          humidity = 83;
          // }
          printLatLong();
          voilation = 0;
          publishPayload();
          digitalWrite(BUZZER_PIN, LOW);
        }
        break;
      case 4:
        {
          latitude = 8.882358;
          longitude = 79.543125;
          brightness = 79;
          // if (isnan(humidity) || isnan(temperature)) {
          temperature = 25;
          humidity = 84;
          // }
          printLatLong();
          voilation = 1;
          publishPayload();
          digitalWrite(BUZZER_PIN, HIGH);
        }
        break;
    }
  }
  emsoliotx.loop();
}
