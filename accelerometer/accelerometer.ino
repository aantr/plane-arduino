/*
    Arduino and ADXL345 Accelerometer - 3D Visualization Example 
     by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>  // Wire library - used for I2C communication

int ADXL345 = 0x53; // The ADXL345 sensor I2C address

int X_out, Y_out, Z_out;  // Outputs
float roll,pitch,rollF,pitchF, z, zF;

void setup() {
  Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
 
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // Bit D3 High for measuring enable (8dec -> 0000 1000 binary)
  Wire.endTransmission();
  delay(10);

  /*
  //Off-set Calibration
  //X-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1E);
  Wire.write(1);
  Wire.endTransmission();
  delay(10);
  //Y-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x1F);
  Wire.write(-2);
  Wire.endTransmission();
  delay(10);

  //Z-axis
  Wire.beginTransmission(ADXL345);
  Wire.write(0x20);
  Wire.write(-9);
  Wire.endTransmission();
  delay(10);
  */
}

int sign(int x){
  if (x < 0)return -1;
  if (x > 0) return 1;
  return 0;
}

void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read() | Wire.read() << 8); // X-axis value
  Y_out = ( Wire.read() | Wire.read() << 8); // Y-axis value
  Z_out = ( Wire.read() | Wire.read() << 8); // Z-axis value

  // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  
  roll = atan2(X_out, Z_out) * 180.0 / PI;
  pitch = atan2(Y_out, Z_out) * 180.0 / PI;
  int lower_bound = 50;
  if (pow(X_out, 2) + pow(Z_out, 2) < pow(lower_bound, 2) * 2) roll=0;
  if (pow(Y_out, 2) + pow(Z_out, 2) < pow(lower_bound, 2) * 2) pitch =0;

  /*roll = atan2(Y_out, sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
  pitch = atan2(-1 * X_out, sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;*/


  // Low-pass filter
  rollF = 0.75 * rollF + 0.25 * roll;
  pitchF = 0.75 * pitchF + 0.25 * pitch;
  zF = 0.75 * zF + 0.25 * z;

  Serial.print(rollF);
  Serial.print("/");
  Serial.print(pitchF);
  Serial.print("/");
  Serial.println(zF);
}
