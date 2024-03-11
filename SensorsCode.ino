#include <array>
#include <string>
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
#include "src/MotionSensor.h"
#include "src/LightSensor.h"
#include "src/UltrasonicSensor.h"
#include "src/VibrationSensor.h"
#include "src/SoundSensor.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// These are the handles to the separate threads. Each individual thread will run one of the sensors
TaskHandle_t vibrationThread;
TaskHandle_t motionThread;
TaskHandle_t proximityThread;
TaskHandle_t lightThread;

////////////////////////////////////////////////////////////////////////////////////////
// Bluetooth Parameters, macros for sensor delay from active to nonactive
////////////////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SUBSYSTEM_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define DESCRIPTOR_UUID "00002902-0000-1000-8000-00805f9b34fb"
#define SUBSYSTEM_DESCRIPTOR "3f0d768b-ddfb-4b46-acdb-e10b904ed065"

#define MOTION_BUFFER_SIZE 500
#define MOTION_BUFFER_CUTOFF 480

// Emergency alert time in minutes
// Every n minutes, if there has been no movement detected, an emergency alert will be sent
// Ex. If EMERGENCY_ALERT_TIME == 2, then every 2 minutes, if motion hasn't been detected, an alert notification will be sent to the Android device
#define EMERGENCY_ALERT_TIME 2

BluetoothSerial bluetooth;
BLECharacteristic* characteristic;  // This is a handle to the main system's main data BLE characteristic that the Android app will retrieve it's data from

// This is a handle to the BLE characteristic that the subsystem will write the room's occupation status to
// The main system will use this characteristic in combination with time since last detection to determine if an emergency alert should be sent
BLECharacteristic* subsystemCharacteristic;

// Pointer to BLEAdvertising object that will be used to start and stop the advertising of the main system's BLE service and characteristics as needed
BLEAdvertising* advertising;

// Tracks the number of devices connected to this main system (via BLE)
// By default, there should at least be one BLE connection at all times (the subsystem) and the main system should not be advertising
// When an emergency alert needs to be sent, the main system will start advertising, and the Android device should pick up the main system on it's scan and connect
int connectedDevices = 0;

////////////////////////////////////////////////////////////////////////////////////////
// Pin setup for sensors
////////////////////////////////////////////////////////////////////////////////////////
const int motionSensorInputPin = 14;
const int trigPin = 2;  // ultrasonic sensor trigger pin
const int echoPin = 15; // ultrasonic sensor echo pin
const int vibrationInputPin = 36;

// Right now, the sound sensor isn't used in the main system
const int soundInputPin = 26;
const int soundGatePin = 27;


// Implementing class for BLEServerCallbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override;
  void onDisconnect(BLEServer* server) override;
};

// After the subsystem successfully establishes a connection, the main system will stop advertising to prevent any other connections
// If for some reason the subsystem is disconnected, begin advertising again to allow it to reconnect
void MyServerCallbacks::onConnect(BLEServer* server) {
  connectedDevices++;
  if (advertising != nullptr) {
     Serial.println("A device connected");
     if (connectedDevices >= 1) {
      Serial.println("All devices connected. Stopping BLE advertise...");
      server->getAdvertising()->stop();
     } else {
      server->getAdvertising()->start(); 
     }
     Serial.print("Connected devices: ");
     Serial.println(connectedDevices);
  }
}

// On subsystem disconnect, if there are no connected devices, begin advertising 
void MyServerCallbacks::onDisconnect(BLEServer* server) {
  connectedDevices--;  
  Serial.println("Device disconnected");
  Serial.print("Connected devices: ");
  Serial.println(connectedDevices);
  if (connectedDevices < 1) {
    server->getAdvertising()->start();
  }
}


////////////////////////////////////////////////////////////////////////////////////////
// calibration time (in seconds) for the sensors to be calibrated
////////////////////////////////////////////////////////////////////////////////////////
const int calibrationTime = 30;

////////////////////////////////////////////////////////////////////////////////////////
// Dynamically allocated MotionSensor object which will be used for interacting with the motion sensor
////////////////////////////////////////////////////////////////////////////////////////
MotionSensor* motionSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic sensor objects for interacting with the ultrasonic sensors
////////////////////////////////////////////////////////////////////////////////////////
UltrasonicSensor* ultrasonicSensor;
UltrasonicSensor* secondUltrasonicSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Vibration sensor object for interacting with the vibration sensor
////////////////////////////////////////////////////////////////////////////////////////
VibrationSensor* vibrationSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Light sensor object
////////////////////////////////////////////////////////////////////////////////////////
LightSensor* lightSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Sound sensor object
////////////////////////////////////////////////////////////////////////////////////////
SoundSensor* soundSensor;

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
unsigned int alertMinutes = 0; // this will determine when an alert will be sent (if it's equal to or greater than EMERGENCY_ALERT_TIME

////////////////////////////////////////////////////////////////////////////////////////
// JSON Object we will send to the Bluetooth app. It keeps track of the status as well as time since last detection
////////////////////////////////////////////////////////////////////////////////////////
StaticJsonDocument<300> jsonData;
String stringData;


////////////////////////////////////////////////////////////////////////////////////////
// Motion Buffer class to store last 500 sensor readings from the motion sensor (work in progress)
////////////////////////////////////////////////////////////////////////////////////////

enum class BufferResult {
  NO_ALERT,
  SEND_ALERT
};

class MotionBuffer {
  private:
  int buffer[MOTION_BUFFER_SIZE];
  int current_index = 0;
  bool full = false;
  public:
  void appendReading(int value);
  void clear();
  bool isFull() const;
  BufferResult processBuffer();
};

void MotionBuffer::appendReading(int value) {
  buffer[current_index] = value;
  current_index++;
  if (current_index >= MOTION_BUFFER_SIZE) {
    full = true;
  }
}

void MotionBuffer::clear() {
  for (int i = 0; i < MOTION_BUFFER_SIZE; i++) {
    buffer[i] = int();
  }
  current_index = 0;
}

bool MotionBuffer::isFull() const { return full; }

BufferResult MotionBuffer::processBuffer() {
  int num_high_readings = 0;
  for (int i = 0; i < MOTION_BUFFER_SIZE; i++) {
    switch(buffer[i]) {
      case 0 : {
        break;
      }
      case 1 : {
        num_high_readings++;
        break;
      }
    }
  }
  if (num_high_readings < MOTION_BUFFER_CUTOFF) {
    return BufferResult::SEND_ALERT;
  } else {
    return BufferResult::NO_ALERT;
  }
  clear();
}

MotionBuffer motion_buffer;

////////////////////////////////////////////////////////////////////////////////////////
// BLE Response Class for parsing the response from the subsystem characteristic and setting room occupied status
////////////////////////////////////////////////////////////////////////////////////////
class BLEResponse {
  private:
  std::string previousResponse = "";
  boolean isOccupied = false;
  
  public:
  void parseResponse(std::string response) {
    if (response == std::string("Occupied")) {
      isOccupied = true;
    } else {
      isOccupied = false;
    }
    // When the room occupation status transitions, we want to reset the minutes counter
    if (response != previousResponse) {
      minutes = 0;
      previousResponse = response;
    }
  }
  bool getOccupied() { return isOccupied; }
};

BLEResponse parser;


////////////////////////////////////////////////////////////////////////////////////////
// TempAdvertise interface that can be implemented to determine what the behavior of starting BLE advertising should be
////////////////////////////////////////////////////////////////////////////////////////
class TempAdvertise {
  public:
  virtual void onTempAdvertising(unsigned long milliseconds) = 0;
};


// Class that implements TempAdvertise
class MyTempAdvertise: public TempAdvertise {
  public:
  void onTempAdvertising(unsigned long milliseconds) override;
};

// Start advertising, wait for the specified number of milliseconds, then stop advertising
void MyTempAdvertise::onTempAdvertising(unsigned long milliseconds) {
  advertising->start();
  delay(milliseconds);
  advertising->stop();
}

void startTempAdvertising(TempAdvertise* tempAdvertise) {
  tempAdvertise->onTempAdvertising(60000);    // begin BLE advertising for a full minute
}

MyTempAdvertise* tempAdvertiseCallback = new MyTempAdvertise();


// Helper method that takes a String message to set the main system's characteristic's value
// We convert the message into an array of bytes before setting the value, which will be translated to proper characters by the ESP32 via UTF-8
void sendBluetoothMessage(String message, BLECharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->setValue(data, message.length());
  characteristic->notify();
}

// Tasks/functions for multithreading sensors
void lightTask(void* parameter) {
  while (true) {
    lightSensor->start();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void soundTask(void* parameter) {
  while (true) {
    soundSensor->start();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vibrationTask(void* parameter) {
  while (true) {
    vibrationSensor->start();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


void setup() {
  motionSensor = new MotionSensor(motionSensorInputPin);
  ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
  lightSensor = new LightSensor;
  vibrationSensor = new VibrationSensor(vibrationInputPin);

  lightSensor->setup();
  vibrationSensor->setup();
  light_module.begin();

  // Initializing I2C (the vibration and light sensor uses I2C)
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Starting BLE setup...");

  // Initialize BLE for the main system with the name "ESP32"
  BLEDevice::init("ESP32");

  // Creating the BLEServer that will hold all of the necessary BLE services and characteristics
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  BLEService* service = server->createService(SERVICE_UUID);

  // Creating the main characteristic that will hold all of the sensor data
  characteristic = service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Creating the characteristic for the subsystem that will hold the current room's occupation status
  subsystemCharacteristic = service->createCharacteristic(SUBSYSTEM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  subsystemCharacteristic->setValue("Occupied");

  characteristic->setValue("Test");
  BLEDescriptor descriptor(DESCRIPTOR_UUID);
  uint8_t descriptorValue[] = {0x00, 0x01};
  BLEDescriptor subsystemDescriptor(SUBSYSTEM_DESCRIPTOR);
  uint8_t subsystemDescriptorValue[] = {0x00, 0x01};
  descriptor.setValue(descriptorValue, 2);
  subsystemDescriptor.setValue(subsystemDescriptorValue, 2);
  subsystemCharacteristic->addDescriptor(&subsystemDescriptor);
  characteristic->addDescriptor(&descriptor);

  // Starting the BLE service that holds the two BLECharacteristics
  service->start();

  // Obtain a handle to BLEAdvertising here and start advertising
  advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE setup complete!\n");

  SPI.begin();

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");

  // Start the light and vibration sensors on a separate thread on core 0
  xTaskCreatePinnedToCore(lightTask, "LightTask", 10000, NULL, 0, NULL, 0);
  //xTaskCreatePinnedToCore(soundTask, "SoundTask", 10000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(vibrationTask, "VibrationTask", 10000, NULL, 0, NULL, 0);
}

void loop() {
  // Motion and ultrasonic sensor are running on the main thread (on core 1)
  motionSensor->start();
  ultrasonicSensor->start();

  // Obtain the number of currently "active" sensors
  // Active means that the sensor has individually concluded that there is human presence/motion within the room
  int activeSensors = motionSensor->isActive() + ultrasonicSensor->isActive() + lightSensor->isActive();

  // Read the subsystem characteristic value every loop. The String value could potentially change after a write from the subsystem
  parser.parseResponse(subsystemCharacteristic->getValue());

  
  Serial.print("Light Sensor Baseline: ");
  Serial.println(lightSensor->baseline());

  // Setting the key value pairs for the JSON that will be sent to the main sensor data BLE characteristic
  // Boolean Data
  jsonData["motion"] = motionSensor->isActive();
  jsonData["proximity"] = ultrasonicSensor->isActive();
  jsonData["light"] = lightSensor->isActive();
  jsonData["occupied"] = parser.getOccupied();

  // Numerical Data
  jsonData["lightIntensity"] = lightSensor->lightValue();
  jsonData["distance"] = ultrasonicSensor->distance();
  jsonData["vibrationIntensity"] = vibrationSensor->intensity();
  jsonData["vibrationBaseline"] = vibrationSensor->baseline();
  jsonData["proximityBaseline"] = ultrasonicSensor->baseline();
  jsonData["lightBaseline"] = lightSensor->baseline();
  jsonData["lightOffBaseline"] = lightSensor->baseline_off();

  jsonData["connected_devices"] = connectedDevices;

  if (lightSensor->lightValue() >= 120) {
    jsonData["lightingStatus"] = "Lights on";
  } else {
    jsonData["lightingStatus"] = "Lights off";
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Two or more of the sensors are reading high (person is present)
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  if (activeSensors >= 2) {
    timeOfDetection = millis();
    minutes = 0;
    alertMinutes = 0;

    jsonData["movement_status"] = "Person is present";
    jsonData["lastDetected"] = 0;
    serializeJson(jsonData, stringData);
    sendBluetoothMessage(stringData, characteristic);
    previousMessage = "Person is present";

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // All sensors read low (person is absent or not being detected for an extended period of time)
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {
    lastDetected = ((millis() - timeOfDetection) / 1000); // keeps track of time since last detection in seconds
    jsonData["movement_status"] = "Not present";

    jsonData["lastDetected"] = minutes;
    serializeJson(jsonData, stringData);
    sendBluetoothMessage(stringData, characteristic);
    previousMessage = "Not present";

    // If person isn't detected, keep track of last detection in minutes
    if (lastDetected >= 60) {
      lastDetected = 0;
      timeOfDetection = millis();
      minutes++;
      alertMinutes++;
      jsonData["lastDetected"] = minutes;

      // If this statement evaluates to true, an emergency alert notification will be sent to the Android device
      if (minutes % EMERGENCY_ALERT_TIME == 0 && parser.getOccupied()) {
        // Start advertising and send an alert to the client device
        alertMinutes = 0;
        startTempAdvertising(tempAdvertiseCallback);
      }

      serializeJson(jsonData, stringData);
      sendBluetoothMessage(stringData, characteristic);
    }
  }
  stringData = "";
}
