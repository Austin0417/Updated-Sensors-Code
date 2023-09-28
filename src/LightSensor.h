#define NUM_LIGHT_READINGS 6

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