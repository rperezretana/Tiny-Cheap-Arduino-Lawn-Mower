#include <Arduino.h>

// Pin definitions
int iPinPulse = 3; // Pin connected to the signal output

void setup() {
  pinMode(iPinPulse, OUTPUT);
  tone(iPinPulse, 41667); // Start generating the 41.67 kHz signal
  Serial.begin(9600);
  Serial.println("Started to generate");
}

void loop() {
  // Nothing needs to be done here if the signal is to run continuously
}