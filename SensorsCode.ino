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
#define SUBSYSTEM_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define DESCRIPTOR_UUID "00002902-0000-1000-8000-00805f9b34fb"

#define NUM_LIGHT_READINGS 6
#define NUM_SOUND_READINGS 5
#define PROXIMITY_PAUSE 3000
#define MOTION_PAUSE 3000
#define VIBRATION_PAUSE 3000
#define SOUND_PAUSE 3000
#define SOUND_THRESHOLD 25

BluetoothSerial bluetooth;
BLECharacteristic* characteristic;
BLECharacteristic* subsystemCharacteristic;

////////////////////////////////////////////////////////////////////////////////////////
// Pin setup for sensors
////////////////////////////////////////////////////////////////////////////////////////
const int motionSensorInputPin = 14;
const int trigPin = 2;
const int echoPin = 15;
const int vibrationInputPin = 36;
const int soundInputPin = 26;
const int soundGatePin = 27;



////////////////////////////////////////////////////////////////////////////////////////
// Proximity sensor class definition/parameters
// Trig pin goes to GIOP 2
// Echo pin goes to GIOP 15
////////////////////////////////////////////////////////////////////////////////////////
class UltrasonicSensor {
  private:
    int trigger_pin;
    int echo_pin;
    long duration;
    int distance_;
    int calibratedValue = 0;
    bool isCalibrated = false;
    long unsigned int lastActiveProximity = 0;
    bool proximityLockLow = true;
    bool proximityTakeLowTime;
    long unsigned int proximityLowIn;

    // Boolean that indicates status of the ultrasonic sensor (active or nonactive)
    bool detected = false;

    // Flag that is set to true whenenver the vibration sensor is active
    // When the system is moved, we need to recalibrate the ultrasonic's baseline
    static bool shouldRetakeBaseline;

  public:
    UltrasonicSensor(int trig_pin, int echo_pin) {
      pinMode(trig_pin, OUTPUT);
      pinMode(echo_pin, INPUT);
      trigger_pin = trig_pin;
      this->echo_pin = echo_pin;
    }

    int distance() {
      return distance_;
    }

    bool isActive() {
      return detected;
    }

    static void setBaselineFlag(bool flag) {
      shouldRetakeBaseline = flag;
    }

    long unsigned int getDetectionTime() {
      return lastActiveProximity;
    }

    int baseline() {
      return calibratedValue;
    }

    int calculateDistance() {
      digitalWrite(trigger_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin, LOW);

      duration = pulseIn(echo_pin, HIGH);

      distance_ = duration * 0.034 / 2;
      return distance_;
    }

    void compareCurrentWithBaseline() {
      if (!isCalibrated || shouldRetakeBaseline) {
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
      }
      else {
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

bool UltrasonicSensor::shouldRetakeBaseline = false;

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
        if (currentVibration >= lights_on_baseline_value + 0.3 || currentVibration <= lights_on_baseline_value - 0.3) {
          Serial.println("Sudden vibration");
          detected = true;
          lastDetectionVibration = millis();
          UltrasonicSensor::setBaselineFlag(true);

          // If 5 seconds have passed since the vibraton has detected, conclude that the person's movement has ended
        } else if (detected && millis() - lastDetectionVibration > VIBRATION_PAUSE) {
          detected = false;
          UltrasonicSensor::setBaselineFlag(false);
          Serial.println("Vibration ended");
        }
      }
      delay(50);
    }
};


////////////////////////////////////////////////////////////////////////////////////////
// Sound sensor class definition/parameters
// Sound sensor input pin
////////////////////////////////////////////////////////////////////////////////////////
class SoundSensor {
  private:
    int currentReading;
    bool detected = false;
    bool soundLockLow = true;
    bool soundTakeLowTime;
    long unsigned int soundLowIn;

    // Average reading, obtained from averaging all of the values in the readings array
    float averageReading = 0;

    // Readings array of integers which will be used to calculate an average value (smoothing the analog reading)
    std::array<int, NUM_SOUND_READINGS> readings;

    // Int to keep track of current index within the readings array
    int currentIndex = 0;
    
    // Boolean flag which will be true (set once and stays true) when the readings array is fully populated
    bool startTakingAverage = false;

  public:
    SoundSensor(int inputPin, int gatePin) {
      pinMode(inputPin, INPUT);
      pinMode(gatePin, INPUT);
    }

    void setup() {
      for (int i = 0; i < NUM_SOUND_READINGS; i++) {
        readings[i] = 0;
      }
    }

    float getAverageReading() {
      float sum = 0;
      for (int i = 0; i < NUM_SOUND_READINGS; i++) {
        sum += readings.at(i);
      }
      averageReading = (sum / NUM_SOUND_READINGS);
      return averageReading;
    }

    float value() {
      return averageReading;
    }
    bool isActive() {
      return detected;
    }

    void start() {
      currentReading = analogRead(soundInputPin);
      readings[currentIndex] = currentReading;
      currentIndex++;

      // Array is populated, starting calculating average
      if (currentIndex >= NUM_SOUND_READINGS) {
        currentIndex = 0;
        startTakingAverage = true;
      }
      if (startTakingAverage) {
        getAverageReading();
      }

      if (digitalRead(soundGatePin) == HIGH) {
        Serial.println("Sound detected");
        detected = true;
        if (soundLockLow) {
          soundLockLow = false;
          delay(50);
        }
        soundTakeLowTime = true;
      }
      if (digitalRead(soundGatePin) == LOW) {
        if (soundTakeLowTime) {
          soundLowIn = millis();
          soundTakeLowTime = false;
        }
        if (!soundLockLow && millis() - soundLowIn > SOUND_PAUSE) {
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
    // Integer variable to track when 30 minutes have elapsed, this happens when currentIndex > 5
    int currentIndex = 0;

    // Array to store periodic values when the light status of the room is on
    std::array<float, NUM_LIGHT_READINGS> readings;
    float lights_on_baseline_value = 0;

    // Array to store periodic values when the light status of the room is off
    std::array<float, NUM_LIGHT_READINGS> readings_lights_off;
    float lights_off_baseline_value = 5.0;

    bool lights_on_is_baseline = false;
    bool lights_off_is_baseline = false;
    bool lightsOn = false;
    bool detected = false;

    String lightingStatus = "";
    float currentReading;

    unsigned long lastIncrementTime = 0;

  public:

    float lightValue() {
      return currentReading;
    }

    bool isActive() {
      return detected;
    }

    float baseline() {
      return lights_on_baseline_value;
    }

    float baseline_off() {
      return lights_off_baseline_value;
    }

    void setCurrentReading(float reading) {
      currentReading = reading;
    }

    std::array<float, NUM_LIGHT_READINGS> get_readings_lights_on() {
      return readings;
    }

    std::array<float, NUM_LIGHT_READINGS> get_readings_lights_off() {
      return readings_lights_off;
    }

    void setup() {
      for (int i = 0; i < NUM_LIGHT_READINGS; i++) {
        // Initially populate the array with -1
        readings[i] = -1;
        readings_lights_off[i] = -1;
      }
    }

    float newBaseline(std::array<float, NUM_LIGHT_READINGS> arr) {
      float sum = 0;
      int divisor = NUM_LIGHT_READINGS;
      for (int i = 0; i < NUM_LIGHT_READINGS; i++) {
        if (arr.at(i) < 0) {
          divisor--;
          continue;
        }
        sum += arr.at(i);
      }
      if (divisor == 0) {
        return 0;
      } else {
        return (sum / divisor);
      }
    }

    void start() {
      setCurrentReading(light_module.readLightLevel());
      if (millis() - lastIncrementTime >= 300000) {

        // After 5 minutes and no drastic change in lighting has been detected, store the current reading into float array
        lastIncrementTime = millis();
        if (lightsOn) {
          readings[currentIndex] = currentReading;
        } else {
          readings_lights_off[currentIndex] = currentReading;
        }
        currentIndex++;
        if (currentIndex >= NUM_LIGHT_READINGS) {
          // After 30 minutes have passed, sum all the elements in both arrays and calculate the average, setting it as the new baseline values
          currentIndex = 0;
          float temp1 = lights_on_baseline_value;
          float temp2 = lights_off_baseline_value;
          lights_on_baseline_value = newBaseline(readings);
          lights_off_baseline_value = newBaseline(readings_lights_off);

          // After obtaining the averages from the arrays, clear the existing values and reset all indices to a value of -1
          setup();

          // If either of the new baseline values are equal to 0, keep the old baseline value
          // This happens when the lighting status hasn't changed in the last 30 minutes, so one of the arrays will contain all -1 values
          if (lights_on_baseline_value == 0) {
            lights_on_baseline_value = temp1;
          }
          if (lights_off_baseline_value == 0) {
            lights_off_baseline_value = temp2;
          }
        }
      }

      // 1. Determine if room lights are on or off from the light reading
      // 2. If lights are on, the threshold should be set higher for detection. If lights are  off, threshold should be set lower for detection
      // 3. Could try having 2 baselines (one for lights on, one for lights off)

      if (currentReading >= 120) {
        // If the current light reading is greater than/equal to 120 lux, conclude that lights in the room are on
        lightsOn = true;
        if (!lights_on_is_baseline) {
          delay(1000);
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
        lightsOn = false;
        if (!lights_off_is_baseline) {
          delay(1000);
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
// Enum definition for whether the door is to the left or right of the system
////////////////////////////////////////////////////////////////////////////////////////
enum DoorDirection {
  LEFT,
  RIGHT
};

// Global DoorDirection variable which will be hardcoded depending on placement of the system relative to the door
const DoorDirection direction = DoorDirection::LEFT;

////////////////////////////////////////////////////////////////////////////////////////
// calibration time for the sensors to be calibrated
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

////////////////////////////////////////////////////////////////////////////////////////
// Global boolean variable indicating the occupation status of the room
////////////////////////////////////////////////////////////////////////////////////////
boolean isOccupied = false;

////////////////////////////////////////////////////////////////////////////////////////
// JSON Object we will send to the Bluetooth app. It keeps track of the status as well as time since last detection
////////////////////////////////////////////////////////////////////////////////////////
StaticJsonDocument<300> jsonData;
String stringData;

////////////////////////////////////////////////////////////////////////////////////////
// BLE Response Class for parsing the response from the subsystem characteristic
////////////////////////////////////////////////////////////////////////////////////////
class BLEResponse {
  private:
  std::string response_;
  public:
  BLEResponse(std::string response) : response_(response) {}
  void parseResponse() {
    if (response_ == std::string("Occupied")) {
      isOccupied = true;
    } else {
      isOccupied = false;
    }
  }
  static void parseResponse(std::string response) {
    if (response == std::string("Occupied")) {
      isOccupied = true;
    } else {
      isOccupied = false;
    }
  }
};
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void sendBluetoothMessage(String message, BLECharacteristic* characteristic) {
  uint8_t data[message.length() + 1];
  memcpy(data, message.c_str(), message.length());
  characteristic->setValue(data, message.length());
  characteristic->notify();
}

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

  Wire.begin();

  Serial.begin(115200);
  Serial.println("Starting BLE setup...");
  BLEDevice::init("ESP32");
  BLEServer* server = BLEDevice::createServer();
  BLEService* service = server->createService(SERVICE_UUID);
  characteristic = service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  subsystemCharacteristic = service->createCharacteristic(SUBSYSTEM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  subsystemCharacteristic->setValue("Occupied");
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

  Serial.print("calibrating sensor ");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  xTaskCreatePinnedToCore(lightTask, "LightTask", 10000, NULL, 0, NULL, 0);
  //xTaskCreatePinnedToCore(soundTask, "SoundTask", 10000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(vibrationTask, "VibrationTask", 10000, NULL, 0, NULL, 0);
}

void loop() {
  motionSensor->start();
  ultrasonicSensor->start();
  //  vibrationSensor->start();
  //  lightSensor->start();
  //  soundSensor->start();

  int activeSensors = motionSensor->isActive() + ultrasonicSensor->isActive() + lightSensor->isActive();

  // Read the subsystem characteristic value every loop. The String value could potentially change after a write from the subsystem
  BLEResponse::parseResponse(subsystemCharacteristic->getValue());

  // Boolean Data
  jsonData["motion"] = motionSensor->isActive();
  jsonData["proximity"] = ultrasonicSensor->isActive();
  jsonData["light"] = lightSensor->isActive();
  jsonData["occupied"] = isOccupied;

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

    //    if (previousMessage != "Person is present") {
    //        jsonData["status"] = "Person is present";
    //        jsonData["lastDetected"] = 0;
    //        serializeJson(jsonData, stringData);
    //        sendBluetoothMessage(stringData, characteristic);
    //        previousMessage = "Person is present";
    //    }

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

    // This if statement will be called when first transitioning from "Person detected" to "Not present"
    // Commented out for testing
    //    if (previousMessage != "Not Present") {
    //        jsonData["lastDetected"] = minutes;
    //        serializeJson(jsonData, stringData);
    //        sendBluetoothMessage(stringData, characteristic);
    //        Serial.println(stringData);
    //        previousMessage = "Not present";
    //        appendFile("data.txt", stringData.c_str());
    //    }

    jsonData["lastDetected"] = minutes;
    serializeJson(jsonData, stringData);
    sendBluetoothMessage(stringData, characteristic);
    previousMessage = "Not present";

    // If person isn't detected, keep track of last detection in minutes
    if (lastDetected >= 60) {
      lastDetected = 0;
      timeOfDetection = millis();
      minutes++;
      jsonData["lastDetected"] = minutes;
      serializeJson(jsonData, stringData);
      sendBluetoothMessage(stringData, characteristic);
    }
  }
  stringData = "";
}
