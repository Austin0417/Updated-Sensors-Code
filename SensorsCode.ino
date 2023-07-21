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
// Bluetooth Parameters, macros for sensor delay from active to nonactive
////////////////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DESCRIPTOR_UUID "00002902-0000-1000-8000-00805f9b34fb"

#define NUM_LIGHT_READINGS 6
#define PROXIMITY_PAUSE 3000
#define MOTION_PAUSE 3000
#define VIBRATION_PAUSE 3000
#define SOUND_PAUSE 3000
#define SOUND_THRESHOLD 25

BluetoothSerial bluetooth;
BLECharacteristic* characteristic;

////////////////////////////////////////////////////////////////////////////////////////
// Pin setup for sensors
////////////////////////////////////////////////////////////////////////////////////////
const int motionSensorInputPin = 14;
const int trigPin = 2;
const int echoPin = 15;
const int vibrationInputPin = 36;
const int soundInputPin = 13;
const int soundGatePin = 12;

////////////////////////////////////////////////////////////////////////////////////////
// Vibration sensor class definition/parameters
// ADXL345 SDA pin goes to GIOP21
// ADXL345 SCL pin goes to GIOP 22
////////////////////////////////////////////////////////////////////////////////////////
class VibrationSensor {
  private:
    Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
    bool vibrationBaselined = false;
    float lights_on_baseline_value = 0.0;
    int lastDetectionVibration = 0;
    bool detected = false;
    float currentVibration;
  public:
    VibrationSensor(int inputPin) {
      pinMode(inputPin, INPUT);
    }

    float baseline() {
      return lights_on_baseline_value;
    }

    bool isActive() {
      return detected;
    }

    float intensity() {
      return currentVibration;
    }

    void setup() {
      if (!accel.begin()) {
        Serial.println("Couldn't detect ADXL345!");
      }
      accel.setRange(ADXL345_RANGE_2_G);
    }

    void start() {
      sensors_event_t event;
      accel.getEvent(&event);

      float x = event.acceleration.x;
      float y = event.acceleration.y;
      float z = event.acceleration.z;
      currentVibration = sqrt((x * x) + (y * y) + (z * z));

      if (!vibrationBaselined) {
        lights_on_baseline_value = currentVibration;
        vibrationBaselined = true;
        Serial.println(lights_on_baseline_value);
      } else {
        if (currentVibration >= lights_on_baseline_value + 0.15 || currentVibration <= lights_on_baseline_value - 0.15) {
          Serial.println("Sudden vibration");
          detected = true;
          lastDetectionVibration = millis();

          // If 5 seconds have passed since the vibraton has detected, conclude that the person's movement has ended
        } else if (detected && millis() - lastDetectionVibration > VIBRATION_PAUSE) {
          detected = false;
          Serial.println("Vibration ended");
        }
      }
      delay(50);
    }
};

////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor class definition/parameters
// Trig pin goes to GIOP 2
// Echo pin goes to GIOP 15
////////////////////////////////////////////////////////////////////////////////////////
class UltrasonicSensor {
  private:
    long duration;
    int distance_;
    int calibratedValue = 0;
    bool isCalibrated = false;
    unsigned int lastActiveProximity = 0;
    bool proximityLockLow = true;
    bool proximityTakeLowTime;
    long unsigned int proximityLowIn;
    bool detected;

  public:
    UltrasonicSensor(int trig_pin, int echo_pin) {
      pinMode(trig_pin, OUTPUT);
      pinMode(echo_pin, INPUT);
    }

    int distance() {
      return distance_;
    }

    bool isActive() {
      return detected;
    }

    int baseline() {
      return calibratedValue;
    }

    int calculateDistance() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);

      distance_ = duration * 0.034 / 2;
      return distance_;
    }

    void compareCurrentWithBaseline() {
      if (!isCalibrated) {
        calibratedValue = distance_;
        isCalibrated = true;
        Serial.println("Calibrated value is: ");
        Serial.print(calibratedValue);
        Serial.print(" cm");
      }
      if (distance_ < calibratedValue - 30 || distance_ > calibratedValue + 30) {
        lastActiveProximity = millis();
        detected = true;
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
        if (!proximityLockLow && millis() - proximityLowIn > PROXIMITY_PAUSE) {
          proximityLockLow = true;
          detected = false;
          Serial.println("Proximity ended");
        }
      }
    }

    void start() {
      int currentDistance = calculateDistance();
      compareCurrentWithBaseline();
    }
};

////////////////////////////////////////////////////////////////////////////////////////
// Sound sensor class definition/parameters
// Sound sensor input pin
////////////////////////////////////////////////////////////////////////////////////////
class SoundSensor {
  private:
    int currentReading;
    bool detected;
    bool soundLockLow = true;
    bool soundTakeLowTime;
    long unsigned int soundLowIn;

  public:
    SoundSensor(int inputPin, int gatePin) {
      pinMode(inputPin, INPUT);
      pinMode(gatePin, INPUT);
    }

    int value() {
      return currentReading;
    }
    bool isActive() {
      return detected;
    }

    void start() {
      currentReading = analogRead(soundInputPin);
      Serial.print("Sound Level: ");
      Serial.println(currentReading);
      if (digitalRead(soundGatePin) == HIGH && currentReading >= SOUND_THRESHOLD) {
        detected = true;
        if (soundLockLow) {
          soundLockLow = false;
          delay(50);
        }
        soundTakeLowTime = true;
      } else {
        if (soundTakeLowTime) {
          soundLowIn = millis();
          soundTakeLowTime = false;
        }
        if (!soundLockLow && millis() - soundLowIn >= SOUND_PAUSE) {
          soundLockLow = true;
          detected = false;
        }
      }
    }
};

////////////////////////////////////////////////////////////////////////////////////////
// Light Sensor class definition/parameters
// BH1750 SDA pin goes to GIOP 21
// BH1750 SCL pin goes to GIOP 22
////////////////////////////////////////////////////////////////////////////////////////
BH1750 light_module;
class LightSensor {
  private:
    int currentIndex = 0;
    float readings[NUM_LIGHT_READINGS];
    bool lights_on_is_baseline = false;
    float lights_on_baseline_value = 0;
    bool lights_off_is_baseline = false;
    float lights_off_baseline_value = 5.0;
    String lightingStatus = "";
    float currentReading;
    bool detected = false;
    unsigned long lastIncrementTime = 0;

  public:

    float lightValue() { return currentReading; }

    bool isActive() { return detected; }

    float baseline() { return lights_on_baseline_value; }

    float baseline_off() { return lights_off_baseline_value; }

    String getLightingStatus() { return lightingStatus; }

    void setCurrentReading(float reading) { currentReading = reading; }

    float* getReadingsArray() { return readings; }

    void setup() {
      for (int i = 0; i < NUM_LIGHT_READINGS; i++) {
        // Initially populate the array with zeros
        readings[i] = 0;
      }
    }

    float newBaseline() {
      float sum = 0;
      for (int i = 0; i < NUM_LIGHT_READINGS; i++) {
        sum += readings[i];
      }
      return (sum / NUM_LIGHT_READINGS);
    }

    void start() {
      setCurrentReading(light_module.readLightLevel());
      if (millis() - lastIncrementTime >= 300000 && !detected) {
        
        // After 5 minutes and no drastic change in lighting has been detected, store the current reading into float array
        lastIncrementTime = millis();
        readings[currentIndex] = currentReading;
        currentIndex++;
        if (currentIndex > 5) {
          // After the array has been completely populated, sum all the elements and calculate the average, setting it as the new baseline
          currentIndex = 0;
          lights_on_baseline_value = newBaseline();
        }
      }
      
      // 1. Determine if room lights are on or off from the light reading
      // 2. If lights are on, the threshold should be set higher for detection. If lights are  off, threshold should be set lower for detection
      // 3. Could try having 2 baselines (one for lights on, one for lights off)
      
      if (currentReading >= 120) {
        // If the current light reading is greater than/equal to 120 lux, conclude that lights in the room are on
        lightingStatus = "Lights on";
        if (!lights_on_is_baseline) {
          delay(500);
          lights_on_is_baseline = true;
          lights_on_baseline_value = currentReading;
        }
        if (currentReading >= lights_on_baseline_value + 5.0 || currentReading <= lights_on_baseline_value - 5.0) {
          // If a change in lux readings of around 20 is detected, this indicates someone passed by the light sensor and affected the light readings
          detected = true;
        } else {
          detected = false;
        }
        
      } else {
        // If the current light reading is not greater than or equal to 120 lux, indicates lights are off
        lightingStatus = "Lights off";
        if (!lights_off_is_baseline) {
          delay(500);
          lights_off_is_baseline = true;
          lights_off_baseline_value = currentReading;
        }
        if (currentReading >= lights_off_baseline_value + 1.5 || currentReading <= lights_off_baseline_value - 1.5) {
          // In a room with lights off, a person's body has drastically less of an effect on the light sensor readings, so the threshold for detection is smaller
          detected = true;
        } else {
          detected = false;
        }
      }
      delay(50);
    }
};

////////////////////////////////////////////////////////////////////////////////////////
// Motion sensor class definition/parameters
// Motion Sensor input pin (middle) goes to GIOP 14
////////////////////////////////////////////////////////////////////////////////////////

class MotionSensor {
  private:
    bool detected = false;
    long unsigned int lowIn;
    bool lockLow = true;
    bool takeLowTime;
    unsigned int lastLowMotion = 0;
    unsigned int lastActiveMotion = 0;
  public:
    bool isActive() {
      return detected;
    }

    MotionSensor(int inputPin) {
      pinMode(inputPin, INPUT);
    }

    void start() {
      if (digitalRead(motionSensorInputPin) == HIGH) {
        detected = true;
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
        if (takeLowTime) {
          lowIn = millis();          //save the time of the transition from high to LOW
          takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
        //if the sensor is low for more than the given pause_,
        //we assume that no more motion is going to happen
        if (!lockLow && millis() - lowIn > MOTION_PAUSE) {
          //makes sure this block of code is only executed again after
          //a new motion sequence has been detected
          lockLow = true;
          lastActiveMotion = millis();
          detected = false;
          Serial.print("motion ended at ");      //output
          Serial.print((millis() - MOTION_PAUSE) / 1000);
          Serial.println(" sec");
          delay(50);
        }
      }
    }
};


////////////////////////////////////////////////////////////////////////////////////////
// calibration time for the sensors to be calibrated
////////////////////////////////////////////////////////////////////////////////////////
const int calibrationTime = 30;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track whether motion was recently detected
////////////////////////////////////////////////////////////////////////////////////////
MotionSensor* motionSensor;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track whether an object was detected in front of the ultrasonic sensor recently
////////////////////////////////////////////////////////////////////////////////////////
UltrasonicSensor* ultrasonicSensor;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track if vibration readings indicate a person walking around in the room
////////////////////////////////////////////////////////////////////////////////////////
VibrationSensor* vibrationSensor;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track if light readings indicate a light has been turned on or not
////////////////////////////////////////////////////////////////////////////////////////
LightSensor* lightSensor;

////////////////////////////////////////////////////////////////////////////////////////
// boolean to track if light readings indicate a light has been turned on or not
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

////////////////////////////////////////////////////////////////////////////////////////
// JSON Object we will send to the Bluetooth app. It keeps track of the status as well as time since last detection
////////////////////////////////////////////////////////////////////////////////////////
StaticJsonDocument<300> jsonData;
String stringData;

void sendBluetoothMessage(String message, BLECharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->setValue(data, message.length());
  characteristic->notify();
}

void appendFile(const char* path, const char* message) {
  File file = SD.open(path, FILE_APPEND);
  if (!file) {
//    Serial.println("Couldn't open file!");
    return;
  }
  if (!file.println(message)) {
    Serial.println("Couldn't write to file!");
  }
  file.close();
}

void setup() {
  motionSensor = new MotionSensor(motionSensorInputPin);
  ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
  lightSensor = new LightSensor;
  vibrationSensor = new VibrationSensor(vibrationInputPin);
  soundSensor = new SoundSensor(soundInputPin, soundGatePin);

  lightSensor->setup();
  vibrationSensor->setup();
  light_module.begin();

  Wire.begin();

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
}

void loop() {
  motionSensor->start();
  ultrasonicSensor->start();
  vibrationSensor->start();
  lightSensor->start();

  int activeSensors = motionSensor->isActive() + ultrasonicSensor->isActive() + lightSensor->isActive() + vibrationSensor->isActive();

  // Boolean Data
  jsonData["motion"] = motionSensor->isActive();
  jsonData["proximity"] = ultrasonicSensor->isActive();
  jsonData["light"] = lightSensor->isActive();
  jsonData["vibration"] = vibrationSensor->isActive();

  // Numerical Data
  jsonData["lightIntensity"] = lightSensor->lightValue();
  jsonData["distance"] = ultrasonicSensor->distance();
  jsonData["vibrationIntensity"] = vibrationSensor->intensity();
  jsonData["vibrationBaseline"] = vibrationSensor->baseline();
  jsonData["proximityBaseline"] = ultrasonicSensor->baseline();
  jsonData["lightBaseline"] = lightSensor->baseline();
  jsonData["lightOffBaseline"] = lightSensor->baseline_off();

  if (lightSensor->lightValue() >= 120) {
    jsonData["lightingStatus"] = "Lights on";
  } else {
    jsonData["lightingStatus"] = "Lights off";
  }

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

    sendBluetoothMessage(stringData, characteristic);
    previousMessage = "Person is present";
    appendFile("data.txt", stringData.c_str());

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // All sensors read low (person is absent or not being detected for an extended period of time)
    /////////////////////////////////////////////////////////////////////////////////////////////////////
  } else {
    lastDetected = ((millis() - timeOfDetection) / 1000);
    Serial.print("Last detection in seconds: ");
    Serial.println(lastDetected);
    jsonData["status"] = "Not present";

    // This if statement will be called when first transitioning from "Person detected" to "Not present"

    jsonData["lastDetected"] = minutes;
    serializeJson(jsonData, stringData);

    sendBluetoothMessage(stringData, characteristic);
    Serial.println(stringData);
    previousMessage = "Not present";
    appendFile("data.txt", stringData.c_str());

    // If person isn't detected, keep track of last detection in minutes
    if (lastDetected >= 60) {
      minutes++;
      Serial.println(minutes);
      lastDetected = 0;
      timeOfDetection = millis();
      jsonData["lastDetected"] = minutes;
      serializeJson(jsonData, stringData);
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
