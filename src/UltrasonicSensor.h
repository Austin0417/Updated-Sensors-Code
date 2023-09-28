#define PROXIMITY_PAUSE 3000


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