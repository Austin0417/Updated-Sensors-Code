#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BH1750.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

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
////////////////////////////////////////////////////////////////////////////////////////
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor parameters
////////////////////////////////////////////////////////////////////////////////////////
long duration;
int distance;
int calibratedValue = 0;
bool isCalibrated = false;
unsigned int lastActiveProximity = 0;
boolean proximityLockLow = true;
boolean proximityTakeLowTime;
long unsigned int proximityPause = 15000;
long unsigned int proximityLowIn;

////////////////////////////////////////////////////////////////////////////////////////
// Light Sensor Parameters
////////////////////////////////////////////////////////////////////////////////////////
BH1750 lightSensor;
int currentIndex = 0;
const int numReadings = 6;
float readings[numReadings];
bool isBaseline = false;
float baseline = 0;
int timePeriod = 0;

////////////////////////////////////////////////////////////////////////////////////////
// Motion sensor parameters
////////////////////////////////////////////////////////////////////////////////////////
bool noRecentMotion;
//the time when the sensor outputs a low impulse
long unsigned int lowIn;

////////////////////////////////////////////////////////////////////////////////////////
//the amount of milliseconds the sensor has to be low
//before we assume all motion has stopped
////////////////////////////////////////////////////////////////////////////////////////
long unsigned int pause_ = 15000;
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
  static unsigned long lastIncrementTime = 0;
  // Check if 10 minutes has passed
  if (millis() - lastIncrementTime >= 3000) {
    lastIncrementTime = millis();
    readings[currentIndex] = lux;
    currentIndex++;
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
  if (!isBaseline) {
   baseline = lux;
   isBaseline = true; 
  }
  if (lux >= baseline + 40.0) {
    // Sudden increase in light detected
    lightDetected = true;
  } else if (lux <= baseline - 40.0) {
    lightDetected = true;
    // Sudden decrease in light detected
  } else {
    lightDetected = false;
  }
  
}

void vibrationSensor() {

  sensors_event_t event;
  accel.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float rms = sqrt((x*x) + (y*y) + (z*z));
  Serial.print("Vibration Intensity (g):");
  Serial.println(rms);
  delay(50);

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
      // Checking if the motion sensor was LOW for longer than 15 seconds after a HIGH
      //      Serial.print("No motion detected in the last ");
      //      Serial.print((millis() - lastActiveMotion) / 1000);
      //      Serial.print(" seconds\n");
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

void proximitySensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;
  if (!isCalibrated) {
    calibratedValue = distance;
    isCalibrated = true;
    Serial.println("Calibrated value is: ");
    Serial.print(calibratedValue);
    Serial.print(" cm");
  }

  if (distance < calibratedValue - 5 || distance > calibratedValue + 5) {
    lastActiveProximity = millis();
    proximityDetected = true;
    if (proximityLockLow) {
      //makes sure we wait for a transition to LOW before any further output is made:
      Serial.println("Proximity");
      proximityLockLow = false;
      delay(50);
    }
    proximityTakeLowTime = true;
    //    Serial.print("Person detected at ");
    //    Serial.print(millis() / 1000);
    //    Serial.print(" seconds\n");
    //    Serial.print("Distance is : ");
    //    Serial.print(distance);
    //    Serial.print(" cm\n");
  } else {
    if (proximityTakeLowTime) {
      proximityLowIn = millis();
      proximityTakeLowTime = false;
    }
    if (!proximityLockLow && millis() - proximityLowIn > proximityPause) {
      proximityLockLow = true;
      proximityDetected = false;
      Serial.println("Proximity ended");
      
    }
    //proximityDetected = false;
  }
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

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
}

void loop() {
  //motionSensor();
  //proximitySensor();
  vibrationSensor();
  //lightModule();

  int activeSensors = motionDetected + proximityDetected + lightDetected + vibrationDetected;

  if (activeSensors >= 2) {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Two or more of the sensors are reading high (person is present)
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    timeOfDetection = millis();
    minutes = 0;
    jsonData["status"] = "Person is present";
    jsonData["lastDetected"] = 0;
    serializeJson(jsonData, stringData);
    if (previousMessage != "Person is present") {
      sendBluetoothMessage(stringData, characteristic);
      Serial.println(stringData);
      previousMessage = "Person is present";
    }
    stringData = "";


  } else {
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // All sensors read low (person is absent or not being detected for an extended period of time)
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    lastDetected = ((millis() - timeOfDetection) / 1000);
    //Serial.println(lastDetected);
    jsonData["status"] = "Not present";

    if (previousMessage != "Not present") {
      minutes = 0;
      jsonData["lastDetected"] = minutes;
      serializeJson(jsonData, stringData);
      sendBluetoothMessage(stringData, characteristic);
      Serial.println(stringData);
      previousMessage = "Not present";
    } else if (lastDetected % 60 == 0) {
      minutes = lastDetected / 60;
      jsonData["lastDetected"] = minutes;
      serializeJson(jsonData, stringData);
      Serial.println(stringData);
      sendBluetoothMessage(stringData, characteristic);
    }
    stringData = "";
  }
}
