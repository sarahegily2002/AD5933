#include "AD5933.h"

AD5933 impedanceAnalyzer;

const int relayPin = 7; // Relay control pin

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);     // Built-in LED
  pinMode(relayPin, OUTPUT); // Relay control
  digitalWrite(relayPin, LOW); // Ensure relay is OFF (using constant resistor)

  impedanceAnalyzer.begin();

  Serial.println("AD5933 Impedance Analyzer");
  Serial.println("Calibrating...");

  // Calibration phase (constant resistor connected via NC pin)
  impedanceAnalyzer.calib();

  Serial.println("Calibration complete");
  Serial.print("Gain Factor: ");
  Serial.println(impedanceAnalyzer.GF);
  Serial.print("System Phase: ");
  Serial.println(impedanceAnalyzer.system_phase);
  Serial.println("\nStarting measurements...");

  // 🔁 Switch relay to connect variable resistor
  digitalWrite(relayPin, HIGH); // Activate relay (connect NO side = variable resistor)
  delay(100); // Small delay for relay switching
}

void loop() {
  delay(5000); // Time between readings

  // Perform impedance measurement
  impedanceAnalyzer.measure();

  // Calculate glucose value based on impedance
  impedanceAnalyzer.glucose_measure(impedanceAnalyzer.Result);

  // Print results
  Serial.print("Impedance: ");
  Serial.print(impedanceAnalyzer.Result);
  Serial.print(" ohms, Phase: ");
  Serial.print(impedanceAnalyzer.phase);
  Serial.print(" degrees, Glucose: ");
  Serial.print(impedanceAnalyzer.gluco);
  Serial.println(" mg/dL");

  delay(2000); // Wait before next measurement
}
