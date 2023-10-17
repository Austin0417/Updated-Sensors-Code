#define VIBRATION_PAUSE 3000

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

    float baseline() const {
      return lights_on_baseline_value;
    }

    bool isActive() const {
      return detected;
    }

    float intensity() const {
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
          LightSensor::setBaselineFlag(true);

          // If 5 seconds have passed since the vibraton has detected, conclude that the person's movement has ended
        } else if (detected && millis() - lastDetectionVibration > VIBRATION_PAUSE) {
          detected = false;
          UltrasonicSensor::setBaselineFlag(false);
          LightSensor::setBaselineFlag(false);
          Serial.println("Vibration ended");
        }
      }
      delay(50);
    }
};