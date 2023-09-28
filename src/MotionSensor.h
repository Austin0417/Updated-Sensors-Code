#define MOTION_PAUSE 3000

////////////////////////////////////////////////////////////////////////////////////////
// Motion sensor class definition/parameters
// Motion Sensor input pin (middle) goes to GIOP 14
////////////////////////////////////////////////////////////////////////////////////////

class MotionSensor {
  private:
    int motionSensorInputPin;
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
      motionSensorInputPin = inputPin;
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