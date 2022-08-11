#include "RF24.h"

RF24 radio;
const int RADIO_PIN_TRANSMITTER[2] = {9, 10};

int pincount = 4;
const int pin[4] = {A0, A1, A2, A3};
const int pinLeft = 8;
const int pinRight = A4;

int get_left_button(){
  return digitalRead(pinLeft) ^ 1;
}

int get_right_button(){
  return (analogRead(pinRight) >= 512) ^ 1;
}

void setup() {

  Serial.begin(9600);

  // nrf test
  if (!radio.begin(RADIO_PIN_TRANSMITTER[0], RADIO_PIN_TRANSMITTER[1])) {Serial.println(F("Radio hardware is not responding"));}
  else {Serial.println(F("Radio hardware is ok"));}

  delay(2000);
  
  for (int i=0;i<pincount;i++){
    pinMode(pin[i], INPUT_PULLUP);
  }
  pinMode(pinLeft, INPUT_PULLUP);
  pinMode(pinRight, INPUT_PULLUP);

}

void loop() {
  
  for (int i=0;i<pincount;i++){
    Serial.print(analogRead(pin[i]));
    Serial.print(" ");
  }
  Serial.print(get_left_button());
  Serial.print(" ");
  Serial.print(get_right_button());
  Serial.print(" ");
  Serial.println("");

}
