#include <EEPROM.h>

void setup() {

  Serial.begin(9600);
  while (!Serial) {}
  EEPROM.update(1, 1);
  Serial.println("updated");
}

void loop() {

}
