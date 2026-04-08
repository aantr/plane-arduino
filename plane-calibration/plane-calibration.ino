#include <Servo.h>
#include <Bluepad32.h>

// Define the signal pin and pulse width limits
#define ESC_PIN 6
#define MAX_PULSE 1500  // Maximum throttle signal (µs)
#define MIN_PULSE 1100  // Minimum throttle signal (µs)

Servo myESC;

void setup() {
  Serial.begin(9600);
  myESC.attach(ESC_PIN, MIN_PULSE, MAX_PULSE); // Attach with defined limits

  Serial.println("=== ESC Calibration Start ===");
  Serial.println("Disconnect battery from ESC now.");
  delay(2000);

  // Step 1: Send max throttle signal
  Serial.println("Sending MAX throttle signal (2000µs)...");
  myESC.writeMicroseconds(MAX_PULSE);
  delay(3000); // Wait for user to connect battery

  // --- CRITICAL STEP: CONNECT THE LIPO BATTERY TO THE ESC NOW ---
  Serial.println(">>> CONNECT THE LIPO BATTERY TO THE ESC NOW <<<");
  delay(5000); // Give time for ESC to read max signal and beep

  // Step 2: Send min throttle signal to set low endpoint
  Serial.println("Sending MIN throttle signal (1000µs)...");
  myESC.writeMicroseconds(MIN_PULSE);
  delay(3000); // Wait for final confirmation beeps

  Serial.println("=== Calibration Complete! ===");
  Serial.println("You can now use the Serial Monitor to test.");
  Serial.println("Enter a value between 0 and 100.");
}

void loop() {
  if (Serial.available() > 0) {
    int throttlePercent = Serial.parseInt();
    throttlePercent = constrain(throttlePercent, 0, 100); // Clamp value

    // Map the 0-100% to the 1000-2000µs pulse width
    int pulseWidth = map(throttlePercent, 0, 100, MIN_PULSE, MAX_PULSE);
    myESC.writeMicroseconds(pulseWidth);

    Serial.print("Throttle set to: ");
    Serial.print(throttlePercent);
    Serial.print("% (");
    Serial.print(pulseWidth);
    Serial.println("µs)");
  }
}
