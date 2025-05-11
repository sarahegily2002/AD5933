#include "AD5933.h"

AD5933 impedanceAnalyzer;

const int relayPin = PA5; // Relay control pin

void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT); // Relay control
  digitalWrite(relayPin, LOW); // Ensure relay is OFF (using constant resistor)

  impedanceAnalyzer.begin();

  Serial.println("AD5933 Impedance Analyzer");
  Serial.println("Calibrating...");

  // Calibration phase (constant resistor connected via NC pin)
  impedanceAnalyzer.calib();

  Serial.println("Calibration complete");
  Serial.print("Gain Factor: ");
  Serial.println(impedanceAnalyzer.GF,12);
  Serial.print("System Phase: ");
  Serial.println(impedanceAnalyzer.system_phase);
  Serial.println("\nStarting measurements...");



  // 🔁 Switch relay to connect variable resistor
  digitalWrite(relayPin, HIGH); // Activate relay (connect NO side = variable resistor)
  delay(100); // Small delay for relay switching
}

void loop() {
  delay(2000); // Time between readings

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
    Serial.println(impedanceAnalyzer.R);
  Serial.println("real part");
    Serial.println(impedanceAnalyzer.I);
  Serial.println("imaginary part");
  Serial.print("Gain Factor: ");
    Serial.println(impedanceAnalyzer.GF,10);

  delay(2000); // Wait before next measurement
}
