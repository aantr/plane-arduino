#include <ps5Controller.h>
#include <SPI.h>
#include <LoRa.h>

// определяем номера пинов, используемые трансивером
#define SS 5
#define RST 14
#define DIO0 2

int ps5_r = 255;
int ps5_g = 0;
int ps5_b = 0;

String byteToHex(byte b) {
  char hex[3];
  sprintf(hex, "%02x", b);
  return String(hex);
}

void setup() {
  // ps5 setup
  Serial.begin(115200);
  ps5.begin("48:18:8D:D7:02:C3"); //replace with MAC address of your controller
  Serial.println("Ps5 Ready.");

  // Lora setup
  Serial.println("LoRa Sender");
 
  //настраиваем трансивер
  LoRa.setPins(SS, RST, DIO0);
  
  //замените LoRa.begin(---E-) частотой, которую вы собираетесь использовать 
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(0xF1);
  
  Serial.println("LoRa Initializing OK!");
  
}

void loop() {
  while (ps5.isConnected() == false) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
    Serial.println("PS5 controller not found.");
    delay(300);
  }

  while (ps5.isConnected() == true) {

//    if (ps5.L1()) Serial.println("L1 Button");
//    if (ps5.R1()) Serial.println("R1 Button");
//
//    if (ps5.Share()) Serial.println("Share Button");
//    if (ps5.Options()) Serial.println("Options Button");
//    if (ps5.L3()) Serial.println("L3 Button");
//    if (ps5.R3()) Serial.println("R3 Button");
//
//    if (ps5.PSButton()) Serial.println("PS Button");
//    if (ps5.Touchpad()) Serial.println("Touch Pad Button");
//
//    if (ps5.L2()) {
//      Serial.printf("L2 button at %d\n", ps5.L2Value());
//    }
//    if (ps5.R2()) {
//      Serial.printf("R2 button at %d\n", ps5.R2Value());
//    }
//
//    Serial.printf("Left Stick x at %d\n", ps5.LStickX());
//    Serial.printf("Left Stick y at %d\n", ps5.LStickY());
//    Serial.printf("Right Stick x at %d\n", ps5.RStickX());
//    Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    
//    String inputString = "<80,80,00>";
    String inputString = "<";
    inputString += byteToHex(min(255, max(0, (int) ps5.RStickY() + 0x80)));
    inputString += ",";
    inputString += byteToHex(min(255, max(0, (int) ps5.RStickX() + 0x80)));
    inputString += ",";
    inputString += byteToHex(min(255, max(0, (int) ps5.LStickY() * 2)));
    inputString += ">";
    
    Serial.print("Sending - ");
    Serial.println(inputString);
   
    //Отправляем сообщение
    LoRa.beginPacket();
    LoRa.print(inputString);
    LoRa.endPacket();

    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(50);
  }
}
