#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

const double fs = 100; // Sample Hz
const double fc = 25;  // Cutoff Hz
const double fn = 2 * fc / fs; // Normalized

// create a rolling average to calculate baseline
const unsigned int rollSumLen = fs * 3; // average over 2 seconds
float rollSumInputs[rollSumLen];
float rollSum = 0;
unsigned int idx = 0;

// create small window for beat detection
const unsigned int winLen = 5;
float win[winLen];
unsigned int winIdx = 0;
float upperThreshold = 100;
float lowerThreshold = -75;

// two timers, one for sampling, and one for beat error prevention
Timer<micros> timer = std::round(1e6 / fs);
Timer<millis> beatTimer = 300;

// create a second order butterworth filter (low pass)
auto filter = butter<2>(fn);

void setup() {
    Serial.begin(115200);
    // LO+ to pin 10
    pinMode(10, INPUT);
    // LO- to pin 11
    pinMode(11, INPUT);

    // initialize rollSumInputs to zeros
    for (int i = 0; i < rollSumLen; i++)
        rollSumInputs[i] = 0;

    // initialize window to zeros
    for (int i = 0; i < winLen; i++)
        win[i] = 0;
}

void loop() {
    if (timer) {
        if ((digitalRead(10) == 1) || (digitalRead(11) == 1)) {
            Serial.println('!');
        } else {
            // measure and filter ecg
            float val = filter(analogRead(A0));

            // add ecg value to rollSum
            rollSum += val;

            // if filled average
            if (idx >= rollSumLen) {
                rollSum -= rollSumInputs[idx%rollSumLen];
            }

            // add measurement to input tracker
            rollSumInputs[idx%rollSumLen] = val;
            idx++;

            // calculate rolling average
            float avg = rollSum / rollSumLen;

            // add val to beat window
            win[winIdx % winLen] = val - avg;
            winIdx++;

            // find max and min in beat window
            float max = std::numeric_limits<float>::min();
            float min = std::numeric_limits<float>::max();
            for (int i = 0; i < winLen; i++) {
                if (win[i] < min)
                    min = win[i];
                else if (win[i] > max)
                    max = win[i];
            }
            // if passed thresholds in window len then beat detected
            int beat = 0;
            if ((min <= lowerThreshold) && (max >= upperThreshold) && beatTimer)
                beat = 100;

            // print values to serial
            Serial.print(avg);            Serial.print(",");
            Serial.print(val - avg);      Serial.print(",");
            Serial.print(beat);           Serial.print(",");
            Serial.print(upperThreshold); Serial.print(",");
            Serial.print(lowerThreshold); Serial.print("\n");
        }
    }
}
