/*
This is a heavily modified version of the DIY_PRO_V3_7.ino sketch from
https://github.com/airgradienthq/arduino.

Important: This code is only for the DIY PRO PCB Version 3.7 that has a push
button mounted.

This code supports the original ESP8266 and an ESP32-C3 as the microcontroller
It supports the original CO2, PM, TVOC and temperature/humidity sensors.
In addition, it supports a BME280/BMP280 sensor for air pressure.

Build Instructions:
https://www.airgradient.com/open-airgradient/instructions/diy-pro-v37/

Kits (including a pre-soldered version) are available:
https://www.airgradient.com/open-airgradient/kits/

The codes needs the following libraries installed:
“WifiManager by tzapu, tablatronix” tested with version 2.0.11-beta
“U8g2” by oliver tested with version 2.32.15
"Sensirion I2C SGP41" by Sensation Version 0.1.0
"Sensirion Gas Index Algorithm" by Sensation Version 3.2.1
"SparkFun BME280" Version 2.0.9
"ArduinoJson" Version 6.21.1
"modbus-esp8266" Version 4.1.0

In addition, the AirGradient libraries are needed. At this point, the orignal
version at https://github.com/airgradienthq/arduino does _not_ work with this
code. It is suggested to use the `lalufu/merge` branch at
https://github.com/Lalufu/airgradienthq-arduino/tree/lalufu/merge instead.

At this time, this branch has the following changes on top of the original
code:

- Use the `modbus-esp8266` library for communication with the CO2 sensor
- Support for the ESP32-C3 microcontroller
- Support for returning a float reading for humidity.

Compared with the original sketch, this file has the following functional
changes:

- ESP32 support
- Support for sending data to MQTT, with configurable endpoints and topics
- Configurable HTTP endpoint for sending data via HTTP
- Support for BME/BMP280 sensors for air pressure measurement

Internal changes:

- Use of `ArduinoJSON` for generating and reading all JSON data
- Store configuraton data in LittleFS
- 4 line display with scrollable output for long lines

This code uses platformio as a build system, which will install all
dependencies automatically. Compiling with Arduino Studio is also possible,
but will require manual installation of dependencies.

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License

*/

#include <AirGradient.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#if defined(ESP8266)
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <HTTPClient.h>
#endif
#include <LittleFS.h>
#include <WiFiClient.h>
#include <WiFiManager.h>

#include <EEPROM.h>

//#include "SGP30.h"
#include "time.h"
#include <SparkFunBME280.h>
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CSgp41.h>
#include <VOCGasIndexAlgorithm.h>

#include <U8g2lib.h>

#if defined(ESP8266)
#define BUTTON_PIN D7
#elif defined(ESP32)
#define BUTTON_PIN A4
#else
#error "No pin definitions known for this board"
#endif

AirGradient ag = AirGradient();
SensirionI2CSgp41 sgp41;
VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;
BME280 bme280;
// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;

// for peristent saving and loading
int addr = 0;
byte value;

// Display bottom right
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Replace above if you have display on top left
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset= */ U8X8_PIN_NONE);

// CONFIGURATION START
// These are default values used when not overridden via the Wifi config screen
// set to the endpoint you would like to use
#define DEFAULT_DEVICE_NAME "Airgradient"
#define DEFAULT_APIROOT "http://hw.airgradient.com/"
#define DEFAULT_MQTT_HOST ""
#define DEFAULT_MQTT_TOPIC "airgradient/tele/%s/SENSOR"
#define DEFAULT_MQTT_PORT 1883
#define WIFI_AP_TIMEOUT 120

// set to true to switch from Celcius to Fahrenheit
boolean inF = false;

// PM2.5 in US AQI (default ug/m3)
boolean inUSAQI = false;

// Display Position
boolean displayTop = true;

// set to true if you want to connect to wifi. You have `WIFI_AP_TIMEOUT` (see
// below) seconds to connect. Then it will go into an offline mode.
boolean connectWIFI = true;

// CONFIGURATION END

unsigned long currentMillis = 0;

const int oledInterval = 5000;
unsigned long previousOled = 0;

const int sendToServerInterval = 5000;
unsigned long previoussendToServer = 0;

const int tvocInterval = 1000;
unsigned long previousTVOC = 0;
int TVOC = 0;
int NOX = 0;

const int co2Interval = 5000;
unsigned long previousCo2 = 0;
int Co2 = 0;

const int pmInterval = 5000;
unsigned long previousPm = 0;
int pm10 = 0;
int pm25 = 0;
int pm100 = 0;

bool pressureSensorFound = false;
const int pressureInterval = 2500;
unsigned long previousPressure = 0;
float pressure = 0;

const int tempHumInterval = 2500;
unsigned long previousTempHum = 0;
float temp = 0;
int hum = 0;
float hum_f = 0;

int buttonConfig = 0;
int lastState = LOW;
int currentState;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

// Whether or not the Wifi setup should save the config
bool shouldSaveConfig = false;

// These are the variables tha the program actually uses
struct RuntimeConfig {
    String device_name;
    String http_server;
    String mqtt_host;
    String mqtt_topic;
    String mqtt_expanded_topic;
    String http_post_url;
    unsigned int mqtt_port;
    bool use_http;
    bool use_mqtt;
};
RuntimeConfig runtimeConfig;

String espID;
// ESP ID, used in various places

WiFiClient mqttWifiClient;
MqttClient mqttClient(mqttWifiClient);

// Buffer to hold the serialized output JSON
// 512 bytes should be enough,
char outputJSONBuffer[512];

void generateChipId()
{
#if defined(ESP8266)
  espID = String(ESP.getChipId(), HEX);
#elif defined(ESP32)
  // ESP32 doesn't have a .getChipId() function, 
  do {
    unsigned int chipId = 0;
    for(int i=0; i<17; i=i+8) {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    espID = String(chipId, HEX);
  } while(0);
#endif
}

void saveConfigToFS()
{
    // Try to save the configuration from the WifiManager to
    // flash
    File configFile;
    StaticJsonDocument<256> doc;
    size_t jsonsize;

    doc["http_server"] = runtimeConfig.http_server;
    doc["mqtt_host"] = runtimeConfig.mqtt_host;
    doc["mqtt_topic"] = runtimeConfig.mqtt_topic;
    doc["mqtt_port"] = runtimeConfig.mqtt_port;
    doc["device_name"] = runtimeConfig.device_name;

    if (!LittleFS.begin()) {
        Serial.println("File system init failed");
        return;
    }

    configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
        Serial.println("could not open config file for writing");
        return;
    }

    jsonsize = serializeJson(doc, configFile);
    configFile.close();
    if (jsonsize == 0) {
        Serial.println("Error saving configuration to /config.json");
    } else {
        Serial.println("Saved configuration to /config.json");
    }

    LittleFS.end();
}

void loadConfigFromFS()
{
    // Try to load configuration for HTTP and MQTT endpoints from
    // flash
    bool loadDefaults = false;
    DeserializationError error;
    File configFile;
    StaticJsonDocument<256> doc;

    if (!LittleFS.begin()) {
        Serial.println("File system init failed");
        loadDefaults = true;
        goto exit;
    }

    if (!LittleFS.exists("/config.json")) {
        Serial.println("config file not found");
        loadDefaults = true;
        goto exit;
    }

    configFile = LittleFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("could not open config file");
        loadDefaults = true;
        goto exit;
    }

    error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        Serial.println("failed to parse JSON");
        loadDefaults = true;
        goto exit;
    }

exit:
    if (loadDefaults) {
        Serial.println("Loading defaults");
        runtimeConfig.device_name = DEFAULT_DEVICE_NAME;
        runtimeConfig.http_server = DEFAULT_APIROOT;
        runtimeConfig.mqtt_host = DEFAULT_MQTT_HOST;
        runtimeConfig.mqtt_topic = DEFAULT_MQTT_TOPIC;
        runtimeConfig.mqtt_port = DEFAULT_MQTT_PORT;
    } else {
        Serial.println("Loaded configuration from /config.json");
        runtimeConfig.device_name = String(doc["device_name"] | DEFAULT_DEVICE_NAME);
        runtimeConfig.http_server = String(doc["http_server"] | DEFAULT_APIROOT);
        runtimeConfig.mqtt_host = String(doc["mqtt_host"] | DEFAULT_MQTT_HOST);
        runtimeConfig.mqtt_topic = String(doc["mqtt_topic"] | DEFAULT_MQTT_TOPIC);
        runtimeConfig.mqtt_port = doc["mqtt_port"] | DEFAULT_MQTT_PORT;
    }

    LittleFS.end();
}

void applyConfig()
{
    char mqtt_expanded_topic[256];

    runtimeConfig.use_http = runtimeConfig.http_server.length() != 0;
    runtimeConfig.use_mqtt = runtimeConfig.mqtt_host.length() != 0;

    snprintf(mqtt_expanded_topic,
        sizeof(mqtt_expanded_topic),
        runtimeConfig.mqtt_topic.c_str(),
        espID.c_str());
    runtimeConfig.mqtt_expanded_topic = String(mqtt_expanded_topic);
    String mqttPort = String(runtimeConfig.mqtt_port);

    if (!runtimeConfig.http_server.endsWith("/")) {
        // Make sure base URL ends with a /
        runtimeConfig.http_server += "/";
    }

    runtimeConfig.http_post_url = runtimeConfig.http_server + "sensors/airgradient:" + espID + "/measures";

    Serial.println("Final http_server: " + runtimeConfig.http_server);
    Serial.println("Final HTTP POST URL: " + runtimeConfig.http_post_url);
    Serial.println("Final mqtt_host: " + runtimeConfig.mqtt_host);
    Serial.println("Final mqtt_topic: " + runtimeConfig.mqtt_topic);
    Serial.println("Final mqtt_expanded_topic: " + runtimeConfig.mqtt_expanded_topic);
    Serial.println("Final mqtt_port: " + mqttPort);
    Serial.println("HTTP enabled: " + String(runtimeConfig.use_http));
    Serial.println("MQTT enabled: " + String(runtimeConfig.use_mqtt));

    if (runtimeConfig.use_http) {
        updateOLED2("HTTP enabled", runtimeConfig.http_server, "", "");
    } else {
        updateOLED2("HTTP disabled", "", "", "");
    }
    delay(3000);

    if (runtimeConfig.use_mqtt) {
        updateOLED2("MQTT enabled", runtimeConfig.mqtt_host, "Port " + mqttPort, "Topic " + runtimeConfig.mqtt_expanded_topic);
    } else {
        updateOLED2("MQTT disabled", "", "", "");
    }
    delay(3000);
}

void saveConfigCallback()
{
    // Called from WifiManager if config needs to be saved
    // This just sets a boolean that we check for in the actual
    // Wifimanager code
    Serial.println("Should save config from Wifi AP");
    shouldSaveConfig = true;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Airgradient starting");
    Serial.print("Firmware built at ");
    Serial.println(__BUILD_TIMESTAMP);
    generateChipId();

    u8g2.begin();
    // u8g2.setDisplayRotation(U8G2_R0);

    // The display layout is saved in EEPROM instead of the file system,
    // for compatibility
    EEPROM.begin(512);
    delay(500);

    buttonConfig = String(EEPROM.read(addr)).toInt();
    setConfig();

    updateOLED2("Press Button", "Now for", "Config Menu", "");
    delay(2000);

    // Initialize the file system and load the configuration
    loadConfigFromFS();

    currentState = digitalRead(BUTTON_PIN);
    if (currentState == HIGH) {
        updateOLED2("Entering", "Config Menu", "", "");
        delay(3000);
        lastState = LOW;
        inConf();
    }

    if (connectWIFI) {
        connectToWifi();
    }

    applyConfig();

    if (runtimeConfig.use_mqtt) {
        mqttClient.setId("airgradient-" + espID);
    }

    updateOLED2("Warming up the", "sensors.", "", "");
    sgp41.begin(Wire);
    ag.CO2_Init();
    ag.PMS_Init();
    ag.TMP_RH_Init(0x44);
    bme280_init();
}

void loop()
{
    currentMillis = millis();
    updateTVOC();
    updateOLED();
    updateCo2();
    updatePm();
    updateTempHum();
    updatePressure();
    sendToServer();
}

void bme280_init()
{
    // Depending on how the sensor is set up the base address
    // can be 0x76 or 0x77. Use the first one we find
    uint16_t baseAddresses[] = { 0x76, 0x77 };

    for (const auto base : baseAddresses) {
        bme280.setI2CAddress(base);

        Serial.print("Probing for pressure sensor at 0x");
        Serial.println(base, HEX);

        if (!bme280.beginI2C(Wire)) {
            // No sensor found
            Serial.println("No BME280 pressure sensor found");
            continue;
        }

        auto res = bme280.readRegister(BME280_CHIP_ID_REG);
        bme280.setFilter(4);
        bme280.setTempOverSample(16);
        bme280.setPressureOverSample(16);
        bme280.setHumidityOverSample(16);
        bme280.setMode(MODE_NORMAL);

        if (res == 0x60) {
            Serial.println("BME 280 sensor found");
            pressureSensorFound = true;
        } else if (res == 0x56 or res == 0x57 or res == 0x58) {
            Serial.println("BMP 280 sensor found");
            pressureSensorFound = true;
        } else if (res == 0x61) {
            Serial.println("BME 680 sensor found");
            pressureSensorFound = true;
        } else {
            Serial.println("Unidentified chip found at BME I2C address");
        }

        if (pressureSensorFound) {
            Serial.print("Found pressure sensor at 0x");
            Serial.println(base, HEX);
            return;
        }
    }
}

void inConf()
{
    // The only way to leave this loop is by restarting the device
    do {
        setConfig();
        currentState = digitalRead(BUTTON_PIN);

        if (lastState == LOW && currentState == HIGH) {
            pressedTime = millis();
        }

        else if (lastState == HIGH && currentState == LOW) {
            releasedTime = millis();
            long pressDuration = releasedTime - pressedTime;
            if (pressDuration < 1000) {
                buttonConfig = buttonConfig + 1;
                if (buttonConfig > 7)
                    buttonConfig = 0;
            }
        }

        if (lastState == HIGH && currentState == HIGH) {
            long passedDuration = millis() - pressedTime;
            if (passedDuration > 4000) {
                updateOLED2("Saved", "Release", "Button Now", "");
                delay(1000);
                updateOLED2("Rebooting", "in", "5 seconds", "");
                delay(5000);
                EEPROM.write(addr, char(buttonConfig));
                EEPROM.commit();
                delay(1000);
                ESP.restart();
            }
        }
        lastState = currentState;
        delay(100);
    } while (1);
}

void setConfig()
{
    if (buttonConfig == 0) {
        updateOLED2("Temp. in C", "PM in ug/m3", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R2);
        inF = false;
        inUSAQI = false;
    }
    if (buttonConfig == 1) {
        updateOLED2("Temp. in C", "PM in US AQI", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R2);
        inF = false;
        inUSAQI = true;
    }
    if (buttonConfig == 2) {
        updateOLED2("Temp. in F", "PM in ug/m3", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R2);
        inF = true;
        inUSAQI = false;
    }
    if (buttonConfig == 3) {
        updateOLED2("Temp. in F", "PM in US AQI", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R2);
        inF = true;
        inUSAQI = true;
    }
    if (buttonConfig == 4) {
        updateOLED2("Temp. in C", "PM in ug/m3", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R0);
        inF = false;
        inUSAQI = false;
    }
    if (buttonConfig == 5) {
        updateOLED2("Temp. in C", "PM in US AQI", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R0);
        inF = false;
        inUSAQI = true;
    }
    if (buttonConfig == 6) {
        updateOLED2("Temp. in F", "PM in ug/m3", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R0);
        inF = true;
        inUSAQI = false;
    }
    if (buttonConfig == 7) {
        updateOLED2("Temp. in F", "PM in US AQI", "Display Top", "");
        u8g2.setDisplayRotation(U8G2_R0);
        inF = true;
        inUSAQI = true;
    }
}

void updateTVOC()
{
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;
    uint16_t compensationRh = 0; // in ticks as defined by SGP41
    uint16_t compensationT = 0; // in ticks as defined by SGP41

    delay(1000);

    compensationT = static_cast<uint16_t>((temp + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(hum * 65535 / 100);

    if (conditioning_s > 0) {
        sgp41.executeConditioning(compensationRh, compensationT, srawVoc);
        conditioning_s--;
    } else {
        sgp41.measureRawSignals(compensationRh, compensationT, srawVoc, srawNox);
    }

    if (currentMillis - previousTVOC >= tvocInterval) {
        previousTVOC += tvocInterval;
        TVOC = voc_algorithm.process(srawVoc);
        NOX = nox_algorithm.process(srawNox);
        Serial.print("TVOC = ");
        Serial.println(String(TVOC));
    }
}

void updateCo2()
{
    if (currentMillis - previousCo2 >= co2Interval) {
        previousCo2 += co2Interval;
        Co2 = ag.getCO2_Raw();
        Serial.print("CO2 = ");
        Serial.println(String(Co2));
    }
}

void updatePm()
{
    if (currentMillis - previousPm >= pmInterval) {
        previousPm += pmInterval;
        pm10 = ag.getPM1_Raw();
        pm25 = ag.getPM2_Raw();
        pm100 = ag.getPM10_Raw();
        Serial.print("pm10 = ");
        Serial.println(String(pm10));
        Serial.print("pm25 = ");
        Serial.println(String(pm25));
        Serial.print("pm100 = ");
        Serial.println(String(pm100));
    }
}

void updateTempHum()
{
    if (currentMillis - previousTempHum >= tempHumInterval) {
        previousTempHum += tempHumInterval;
        TMP_RH result = ag.periodicFetchData();
        temp = result.t;
        hum = result.rh;
        hum_f = result.rh_f;
        Serial.print("Temp = ");
        Serial.println(String(temp));
    }
}

void updatePressure()
{
    BME280_SensorMeasurements measurement;
    if (!pressureSensorFound) {
        return;
    }
    if (currentMillis - previousPressure >= pressureInterval) {
        while(bme280.isMeasuring()) {
            delay(1);
        }
        bme280.readAllMeasurements(&measurement);
        Serial.print("BME280 Humidity: ");
        Serial.println(measurement.humidity);
        Serial.print("BME280 Temperature: ");
        Serial.println(measurement.temperature);
        Serial.print("BME280 Pressure: ");
        Serial.println(measurement.pressure);

        // Pressure as returned from this call is in Pa,
        // but we want hPa
        pressure = measurement.pressure/100; 
    }
}

void updateOLED()
{
    if (currentMillis - previousOled >= oledInterval) {
        previousOled += oledInterval;

        String ln1;
        String ln2;
        String ln3;
        String ln4;

        if (inUSAQI) {
            ln1 = "AQI:" + String(PM_TO_AQI_US(pm25)) + " CO2:" + String(Co2);
        } else {
            ln1 = "PM:" + String(pm25) + " CO2:" + String(Co2);
        }

        ln2 = "TVOC:" + String(TVOC) + " NOX:" + String(NOX);

        if (inF) {
            ln3 = "F:" + String((temp * 9 / 5) + 32) + " H:" + String(hum) + "%";
        } else {
            ln3 = "C:" + String(temp) + " H:" + String(hum) + "%";
        }

        if (pressureSensorFound) {
            int p = pressure;
            ln4 = "Press:" + String(p);
        } else {
            ln4 = "";
        }
        updateOLED2(ln1, ln2, ln3, ln4);
    }
}

void updateOLED2(String ln1, String ln2, String ln3, String ln4)
{
    // This procedure will print the four strings to the display,
    // scrolling lines that are longer than 16 characters
    //
    // When scrolling is necessary this function will not return
    // immediately.

    unsigned int spacing = 17;
    unsigned int lln[4] = { ln1.length(), ln2.length(), ln3.length(), ln4.length() };
    unsigned int maxscroll[4] = { 0, 0, 0, 0 };
    unsigned int scroll[4] = { 0, 0, 0, 0 };
    const char* cstr[4] = { ln1.c_str(), ln2.c_str(), ln3.c_str(), ln4.c_str() };
    bool doScroll = false;
    bool firstLoop = true;

    unsigned short i = 0;
    for (auto len : lln) {
        if (len > 16)
            maxscroll[i] = len - 16;
        i++;
    }

    do {
        u8g2.firstPage();
        u8g2.firstPage();
        if (doScroll) {
            if (firstLoop) {
                delay(2000);
            } else {
                delay(250);
            }
            doScroll = false;
            firstLoop = false;
        }
        do {
            u8g2.setFont(u8g2_font_t0_16_tf);
            for (auto i : { 0, 1, 2, 3 }) {
                u8g2.drawStr(1, 10 + (i * spacing), cstr[i] + scroll[i]);
                if (scroll[i] < maxscroll[i]) {
                    doScroll = true;
                    scroll[i]++;
                }
            }
        } while (u8g2.nextPage());
    } while (doScroll);
}

void sendToServer()
{
    time_t now;
    StaticJsonDocument<512> outputJSON;
    size_t jsonSize;

    now = time(nullptr);

    if (now < __BUILD_TIMESTAMP) {
        // This is in the past as of the creation of this code
        Serial.println("Time not syncronized?");
        return;
    }

    if (!(runtimeConfig.use_mqtt or runtimeConfig.use_http)) {
        // No outputs enabled
        Serial.println("No output servers enabled");
        return;
    }

    if (runtimeConfig.use_mqtt) {
        mqttClient.poll();
    }

    if (currentMillis - previoussendToServer >= sendToServerInterval) {
        previoussendToServer += sendToServerInterval;
        // The output JSON is produced in two stages. The first
        // stage produces a document suitable for sending to a HTTP endpoint,
        // the second stage adds more fields for sending data to the MQTT
        // endpoint
        outputJSON["wifi"] = WiFi.RSSI();
        outputJSON["atmp"] = temp;
        if (Co2 >= 0)
            outputJSON["rco2"] = Co2;
        if (pm25 >= 0)
            outputJSON["pm02"] = pm25;
        if (TVOC >= 0)
            outputJSON["tvoc_index"] = TVOC;
        if (NOX >= 0)
            outputJSON["nox_index"] = NOX;
        if (hum >= 0)
            outputJSON["rhum"] = hum;

        if (WiFi.status() == WL_CONNECTED) {
            if (runtimeConfig.use_http) {
                jsonSize = serializeJson(outputJSON, outputJSONBuffer, sizeof(outputJSONBuffer));
                if (jsonSize >= sizeof(outputJSONBuffer)) {
                    // Possible overflow, or terminating \0
                    // not written
                    Serial.println("Serialized JSON document too large");
                    return;
                }
                Serial.print("HTTP JSON length ");
                Serial.println(jsonSize);
                Serial.print("HTTP JSON = ");
                Serial.println(outputJSONBuffer);
                WiFiClient client;
                HTTPClient http;
                http.begin(client, runtimeConfig.http_post_url);
                http.addHeader("content-type", "application/json");
                int httpCode = http.POST((uint8_t*)outputJSONBuffer, jsonSize - 1);
                String response = http.getString();
                Serial.print("HTTP return code = ");
                Serial.println(httpCode);
                Serial.print("HTTP response = ");
                Serial.println(response);
                http.end();
            }

            if (runtimeConfig.use_mqtt) {
                // Add more fields for MQTT
                outputJSON["device_name"] = runtimeConfig.device_name;
                outputJSON["espid"] = espID;
                outputJSON["time"] = now;
                outputJSON["aqi_us"] = PM_TO_AQI_US(pm25);
                outputJSON["rhum"] = hum_f;
                outputJSON["dewpoint"] = dewpoint(temp, hum_f);
                outputJSON["pm01"] = pm10;
                outputJSON["pm10"] = pm100;
                if (pressure > 0)
                    outputJSON["pres"] = pressure;
                jsonSize = serializeJson(outputJSON, outputJSONBuffer, sizeof(outputJSONBuffer));
                if (jsonSize >= sizeof(outputJSONBuffer)) {
                    // Possible overflow, or terminating \0
                    // not written
                    Serial.println("Serialized JSON document too large");
                    return;
                }
                Serial.print("MQTT JSON length ");
                Serial.println(jsonSize);
                Serial.print("MQTT JSON = ");
                Serial.println(outputJSONBuffer);

                if (!mqttClient.connected()) {
                    Serial.println("Attempting to connect to MQTT");
                    // Attempt to connect
                    if (!mqttClient.connect(runtimeConfig.mqtt_host.c_str(),
                            runtimeConfig.mqtt_port)) {
                        Serial.print("MQTT connection failed. Error code = ");
                        Serial.println(mqttClient.connectError());
                    } else {
                        Serial.println("Connection successful");
                    }
                } else {
                    // Connected, send data
                    Serial.println("Sending to MQTT");
                    mqttClient.beginMessage(runtimeConfig.mqtt_expanded_topic);
                    mqttClient.print(outputJSONBuffer);
                    mqttClient.endMessage();
                }
            }
        } else {
            Serial.println("WiFi Disconnected");
        }
    }
}

// Wifi Manager
void connectToWifi()
{
    WiFiManager wifiManager;
    // WiFi.disconnect(); //to delete previous saved hotspot
    String HOTSPOT = "AG-" + espID;
    updateOLED2(
        String(WIFI_AP_TIMEOUT) + "s to connect", "to Wifi Hotspot", HOTSPOT, "");
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.setTimeout(WIFI_AP_TIMEOUT);

    WiFiManagerParameter device_name_value(
        "device_name",
        "Device name. Will be sent in MQTT messages.",
        runtimeConfig.device_name.c_str(),
        255);
    wifiManager.addParameter(&device_name_value);

    WiFiManagerParameter http_server_value(
        "http_server",
        "HTTP server to send measurements to<br>Leave blank to disable HTTP",
        runtimeConfig.http_server.c_str(),
        255);
    wifiManager.addParameter(&http_server_value);

    WiFiManagerParameter mqtt_host_value(
        "mqtt_host",
        "MQTT host to send measurements to<br>Leave blank to disable MQTT",
        runtimeConfig.mqtt_host.c_str(),
        255);
    wifiManager.addParameter(&mqtt_host_value);

    WiFiManagerParameter mqtt_topic_value(
        "mqtt_topic",
        "MQTT topic to send measurements to<br>%s will be replaced by the chip ID",
        runtimeConfig.mqtt_topic.c_str(),
        255);
    wifiManager.addParameter(&mqtt_topic_value);

    String portString = String(runtimeConfig.mqtt_port);
    WiFiManagerParameter mqtt_port_value(
        "mqtt_port", "MQTT port to send measurements to", portString.c_str(), 5);
    wifiManager.addParameter(&mqtt_port_value);

    if (!wifiManager.autoConnect((const char*)HOTSPOT.c_str())) {
        updateOLED2("booting into", "offline mode", "", "");
        Serial.println("failed to connect and hit timeout");
        delay(6000);
        return;
    }
    // Connected to Wifi, attempt to save the config if required
    if (shouldSaveConfig) {
        long int parsedPort;

        runtimeConfig.http_server = String(http_server_value.getValue());
        runtimeConfig.mqtt_host = String(mqtt_host_value.getValue());
        runtimeConfig.mqtt_topic = String(mqtt_topic_value.getValue());
        runtimeConfig.device_name = String(device_name_value.getValue());
        parsedPort = strtol(mqtt_port_value.getValue(), NULL, 10);
        if ((parsedPort <= 0) or (parsedPort >= 65536)) {
            // Parsing failure
            Serial.println("Invalid MQTT port value: " + String(mqtt_port_value.getValue()));
            Serial.println("Not saving configuration");
            return;
        }
        runtimeConfig.mqtt_port = parsedPort;

        saveConfigToFS();
    }
}

// Calculate PM2.5 US AQI
int PM_TO_AQI_US(int pm02)
{
    if (pm02 <= 12.0)
        return ((50 - 0) / (12.0 - .0) * (pm02 - .0) + 0);
    else if (pm02 <= 35.4)
        return ((100 - 50) / (35.4 - 12.0) * (pm02 - 12.0) + 50);
    else if (pm02 <= 55.4)
        return ((150 - 100) / (55.4 - 35.4) * (pm02 - 35.4) + 100);
    else if (pm02 <= 150.4)
        return ((200 - 150) / (150.4 - 55.4) * (pm02 - 55.4) + 150);
    else if (pm02 <= 250.4)
        return ((300 - 200) / (250.4 - 150.4) * (pm02 - 150.4) + 200);
    else if (pm02 <= 350.4)
        return ((400 - 300) / (350.4 - 250.4) * (pm02 - 250.4) + 300);
    else if (pm02 <= 500.4)
        return ((500 - 400) / (500.4 - 350.4) * (pm02 - 350.4) + 400);
    else
        return 500;
};

// Calculate dew point temperature
float dewpoint(float temp, float hum)
{
    /*
     * Calculate an approximate dewpoint temperature Tdp, given a temperature T
     * and relative humidity H.
     *
     * This uses the Magnus formula:
     *
     * N = ln(H / 100) + (( b * T ) / ( c + T ))
     *
     * Tdp = ( c * N ) / ( b - N )
     *
     * The constants b and c come from
     *
     * https://doi.org/10.1175/1520-0450(1981)020%3C1527:NEFCVP%3E2.0.CO;2
     *
     * and are
     * b = 17.368
     * c = 238.88
     *
     * for temperatures >= 0 degrees C and
     *
     * b = 17.966
     * c = 247.15
     *
     * for temperatures < 0 degrees C
     */
    float N;
    float b, c;

    if (temp >= 0) {
        b = 17.368;
        c = 238.88;
    } else {
        b = 17.966;
        c = 247.15;
    }

    N = log(hum / 100) + ((b * temp) / (c + temp));

    return (c * N) / (b - N);
}
