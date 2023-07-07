#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BH1750.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

TaskHandle_t vibrationThread;
TaskHandle_t motionThread;
TaskHandle_t proximityThread;
TaskHandle_t lightThread;

////////////////////////////////////////////////////////////////////////////////////////
// Bluetooth Parameters
////////////////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DESCRIPTOR_UUID "00002902-0000-1000-8000-00805f9b34fb"

BluetoothSerial bluetooth;
BLECharacteristic* characteristic;

////////////////////////////////////////////////////////////////////////////////////////
// Pin setup for sensors
////////////////////////////////////////////////////////////////////////////////////////
const int motionSensorInputPin = 14;
const int trigPin = 2;
const int echoPin = 15;
const int vibrationInputPin = 36;

////////////////////////////////////////////////////////////////////////////////////////
// Vibration sensor parameters
// ADXL345 SDA pin goes to GIOP21
// ADXL345 SCL pin goes to GIOP 22
////////////////////////////////////////////////////////////////////////////////////////
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool vibrationBaselined = false;
float baselineVibration = 0.0;
int lastDetectionVibration = 0;
float vibrationIntensity;

////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor parameters
// Trig pin goes to GIOP 2
// Echo pin goes to GIOP 15
////////////////////////////////////////////////////////////////////////////////////////
long duration;
int distance;
int calibratedValue = 0;
bool isCalibrated = false;
unsigned int lastActiveProximity = 0;
boolean proximityLockLow = true;
boolean proximityTakeLowTime;
long unsigned int proximityPause = 3000;
long unsigned int proximityLowIn;

////////////////////////////////////////////////////////////////////////////////////////
// Light Sensor Parameters
// BH1750 SDA pin goes to GIOP 21
// BH1750 SCL pin goes to GIOP 22
////////////////////////////////////////////////////////////////////////////////////////
BH1750 lightSensor;
int currentIndex = 0;
const int numReadings = 6;
float readings[numReadings];
bool isBaseline = false;
float baseline = 0;
int timePeriod = 0;
float lightIntensity;

////////////////////////////////////////////////////////////////////////////////////////
// Motion sensor parameters
// Motion Sensor input pin (middle) goes to GIOP 14
////////////////////////////////////////////////////////////////////////////////////////
bool noRecentMotion;
//the time when the sensor outputs a low impulse
long unsigned int lowIn;

////////////////////////////////////////////////////////////////////////////////////////
//the amount of milliseconds the sensor has to be low
//before we assume all motion has stopped
////////////////////////////////////////////////////////////////////////////////////////
long unsigned int pause_ = 3000;
boolean lockLow = true;
boolean takeLowTime;
unsigned int lastLowMotion = 0;
unsigned int lastActiveMotion = 0;

////////////////////////////////////////////////////////////////////////////////////////
// calibration time for the sensors to be calibrated
////////////////////////////////////////////////////////////////////////////////////////
const int calibrationTime = 30;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track whether motion was recently detected
////////////////////////////////////////////////////////////////////////////////////////
bool motionDetected = false;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track whether an object was detected in front of the ultrasonic sensor recently
////////////////////////////////////////////////////////////////////////////////////////
bool proximityDetected = false;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track if vibration readings indicate a person walking around in the room
////////////////////////////////////////////////////////////////////////////////////////
bool vibrationDetected = false;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track if light readings indicate a light has been turned on or not
////////////////////////////////////////////////////////////////////////////////////////
bool lightDetected = false;

////////////////////////////////////////////////////////////////////////////////////////
// String variable to keep track of previous detection state
////////////////////////////////////////////////////////////////////////////////////////
String previousMessage = "";

////////////////////////////////////////////////////////////////////////////////////////
// keep track of current time in ms when one of the sensors read HIGH
////////////////////////////////////////////////////////////////////////////////////////
unsigned int timeOfDetection = 0;
unsigned int lastDetected = 0;  // time since last detection in seconds
unsigned int minutes = 0;     // minutes that have passed since last detection

////////////////////////////////////////////////////////////////////////////////////////
// JSON Object we will send to the Bluetooth app. It keeps track of the status as well as time since last detection
////////////////////////////////////////////////////////////////////////////////////////
StaticJsonDocument<200> jsonData;
String stringData;

void sendBluetoothMessage(String message, BLECharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->setValue(data, message.length());
  characteristic->notify();
}

void lightModule() {
  float lux = lightSensor.readLightLevel();
  lightIntensity = lux;
  static unsigned long lastIncrementTime = 0;


  // Check if 5 minutes have passed, store the current lux reading into the array
  if (millis() - lastIncrementTime >= 300000 && !lightDetected) {
    lastIncrementTime = millis();
    readings[currentIndex] = lux;
    currentIndex++;

    // After populating the array with 6 values, start computing the average, which will serve as the new baseline
    // Goal is to account for changes in ambient light over the course of a day
    if (currentIndex > numReadings - 1) {
      currentIndex = 0;
      float average = 0;
      for (int i = 0; i < numReadings - 1; i++) {
        average += readings[i];
      }
      average = average / 6.0;
      baseline = average;
    }
  }

  // Identical approach to vibration and proximity sensor, set a baseline value to compare future readings to
  if (!isBaseline) {
    baseline = lux;
    isBaseline = true;
  }

  // Sudden increase in light detected
  if (lux >= baseline + 40.0) {
    lightDetected = true;

    // Sudden decrease in light detected
  } else if (lux <= baseline - 40.0) {
    lightDetected = true;

  } else {
    lightDetected = false;
  }

}

void lightTask(void* parameters) {
  while (true) {
    lightModule();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


void vibrationSensor() {

  sensors_event_t event;
  accel.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float rms = sqrt((x * x) + (y * y) + (z * z));

  vibrationIntensity = rms;

  // Initial setup of baseline value for vibration sensor (units are m/s^2)
  if (!vibrationBaselined) {
    baselineVibration = rms;
    Serial.println(baselineVibration);
    vibrationBaselined = true;

    // After obtaining baseline value, compare current RMS readings with baseline. If they differ by an offset of 0.1,
    // conclude that there is movement in the room/person present in the room
  } else {
    if (rms >= baselineVibration + 0.1 || rms <= baselineVibration - 0.1) {
      Serial.println("Sudden vibration");
      vibrationDetected = true;
      lastDetectionVibration = millis();

      // If 5 seconds have passed since the vibraton has detected, conclude that the person's movement has ended
    } else if (vibrationDetected && millis() - lastDetectionVibration > 5000) {
      vibrationDetected = false;
      Serial.println("Vibration ended");
    }
  }

  delay(50);

}

void vibrationTask(void *parameters) {
  while (true) {
    vibrationSensor();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


void motionSensor() {
  if (digitalRead(motionSensorInputPin) == HIGH) {
    motionDetected = true;
    noRecentMotion = false;
    if (lockLow) {
      //makes sure we wait for a transition to LOW before any further output is made:
      lockLow = false;
      Serial.println("---");
      Serial.print("motion detected at ");
      Serial.print(millis() / 1000);
      Serial.println(" sec");
      delay(50);
    }
    takeLowTime = true;
  }

  if (digitalRead(motionSensorInputPin) == LOW) {
    lastLowMotion = millis();
    if (lastLowMotion - lastActiveMotion > 15000) {
      noRecentMotion = true;
    }
    if (takeLowTime) {
      lowIn = millis();          //save the time of the transition from high to LOW
      takeLowTime = false;       //make sure this is only done at the start of a LOW phase
    }

    //if the sensor is low for more than the given pause_,
    //we assume that no more motion is going to happen
    if (!lockLow && millis() - lowIn > pause_) {
      //makes sure this block of code is only executed again after
      //a new motion sequence has been detected
      lockLow = true;
      lastActiveMotion = millis();
      motionDetected = false;
      Serial.print("motion ended at ");      //output
      Serial.print((millis() - pause_) / 1000);
      Serial.println(" sec");
      delay(50);
    }
  }
}

void motionTask(void* parameters) {
  while (true) {
    motionSensor();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void proximitySensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  // Obtaining baseline value for the proximity sensor, to compare future readings with
  if (!isCalibrated) {
    calibratedValue = distance;
    isCalibrated = true;
    Serial.println("Calibrated value is: ");
    Serial.print(calibratedValue);
    Serial.print(" cm");
  }

  // Comparing the baseline value with the current readings.
  // If the readings differ from the baseline value by + or - 5 cm, this means that someone has walked in front of the proximtiy sensor
  if (distance < calibratedValue - 30 || distance > calibratedValue + 30) {
    lastActiveProximity = millis();
    proximityDetected = true;

    if (proximityLockLow) {
      //makes sure we wait for a transition to LOW before any further output is made:
      Serial.println("Proximity");
      proximityLockLow = false;
      delay(50);
    }
    proximityTakeLowTime = true;

  } else {
    if (proximityTakeLowTime) {
      proximityLowIn = millis();
      proximityTakeLowTime = false;
    }

    // If the proximity sensor has not read HIGH for more than the defined pause time, conclude that the person has left the room
    if (!proximityLockLow && millis() - proximityLowIn > proximityPause) {
      proximityLockLow = true;
      proximityDetected = false;
      Serial.println("Proximity ended");

    }
    //proximityDetected = false;
  }
}

void proximityTask(void* parameters) {
  while (true) {
    proximitySensor();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void appendFile(const char* path, const char* message) {
  File file = SD.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Couldn't open file!");
    return;
  }
  if (!file.println(message)) {
    Serial.println("Couldn't write to file!");
  }
  file.close();
}

void setup() {
  pinMode(motionSensorInputPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(vibrationInputPin, INPUT);
  Wire.begin();
  lightSensor.begin();
  if (!accel.begin()) {
    Serial.println("Couldn't detect ADXL345!");
  }
  accel.setRange(ADXL345_RANGE_2_G);
  Serial.println(accel.getRange());
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  Serial.begin(115200);
  Serial.println("Starting BLE setup...");
  BLEDevice::init("ESP32");
  BLEServer* server = BLEDevice::createServer();
  BLEService* service = server->createService(SERVICE_UUID);
  characteristic = service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  characteristic->setValue("Test");
  BLEDescriptor descriptor(DESCRIPTOR_UUID);
  uint8_t descriptorValue[] = {0x00, 0x01};
  descriptor.setValue(descriptorValue, 2);
  characteristic->addDescriptor(&descriptor);
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE setup complete!\n");

  SPI.begin();
  if (SD.begin(5)) {
    Serial.println("SD card setup success");
  }

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  //  xTaskCreatePinnedToCore(vibrationTask, "Vibration task", 10000, NULL, 1, &vibrationThread, 0);
  //  xTaskCreatePinnedToCore(motionTask, "Motion task", 10000, NULL, 1, &motionThread, 0);
  //  xTaskCreatePinnedToCore(proximityTask, "Proximity task", 10000, NULL, 1, &proximityThread, 1);
  //  xTaskCreatePinnedToCore(lightTask, "Light task", 10000, NULL, 1, &lightThread, 1);
}

void loop() {
  motionSensor();
  proximitySensor();
  vibrationSensor();
  lightModule();

  int activeSensors = motionDetected + proximityDetected + lightDetected + vibrationDetected;

  // Boolean Data
  jsonData["motion"] = motionDetected;
  jsonData["proximity"] = proximityDetected;
  jsonData["light"] = lightDetected;
  jsonData["vibration"] = vibrationDetected;

  // Numerical Data
  jsonData["lightIntensity"] = lightIntensity;
  jsonData["distance"] = distance;
  jsonData["vibrationIntensity"] = vibrationIntensity;
  jsonData["vibrationBaseline"] = baselineVibration;
  jsonData["proximityBaseline"] = calibratedValue;
  jsonData["lightBaseline"] = baseline;

  static int lastSamplingTime = 0;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Two or more of the sensors are reading high (person is present)
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  if (activeSensors >= 2) {
    timeOfDetection = millis();
    minutes = 0;
    jsonData["status"] = "Person is present";
    jsonData["lastDetected"] = 0;
    serializeJson(jsonData, stringData);
    Serial.println("Two or more sensors");
    sendBluetoothMessage(stringData, characteristic);
    Serial.println(stringData);
    previousMessage = "Person is present";
    appendFile("data.txt", stringData.c_str());

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // All sensors read low (person is absent or not being detected for an extended period of time)
    /////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {
    lastDetected = ((millis() - timeOfDetection) / 1000);
    jsonData["status"] = "Not present";

    // This if statement will be called when first transitioning from "Person detected" to "Not present"

    Serial.println("No detection");
    minutes = 0;
    jsonData["lastDetected"] = minutes;
    serializeJson(jsonData, stringData);
    sendBluetoothMessage(stringData, characteristic);
    Serial.println(stringData);
    previousMessage = "Not present";
    appendFile("data.txt", stringData.c_str());

    // If person isn't detected, keep track of last detection in minutes
    if (lastDetected % 60 == 0) {
      minutes = lastDetected / 60;
      jsonData["lastDetected"] = minutes;
      serializeJson(jsonData, stringData);
      Serial.println(stringData);
      sendBluetoothMessage(stringData, characteristic);

    }
  }
  if (millis() - lastSamplingTime >= 600000) {
    lastSamplingTime = millis();
    sendBluetoothMessage(stringData, characteristic);
    appendFile("data.txt", stringData.c_str());
  }
  stringData = "";
}
