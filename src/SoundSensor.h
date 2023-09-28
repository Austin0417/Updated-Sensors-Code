#define SOUND_PAUSE 3000
#define SOUND_THRESHOLD 25
#define NUM_SOUND_READINGS 5


////////////////////////////////////////////////////////////////////////////////////////
// Sound sensor class definition/parameters
// Sound sensor input pin
////////////////////////////////////////////////////////////////////////////////////////
class SoundSensor {
  private:
    int currentReading;
    int soundInputPin;
    int soundGatePin;
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
      soundInputPin = inputPin;
      soundGatePin = gatePin;
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