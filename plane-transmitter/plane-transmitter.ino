#include <SPI.h>
#include <LoRa.h>
 
//определяем номера пинов, используемые трансивером
#define ss 5
#define rst 14
#define dio0 2
 
int counter = 0;
 
void setup() {
  //запускаем монитор порта
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");
 
  //настраиваем трансивер
  LoRa.setPins(ss, rst, dio0);
  
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

  if (Serial.available() > 0) {
    // Читаем всю строку до символа новой строки (\n)
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); // Удаляем пробелы и символы перевода строки

    Serial.print("Sending packet: ");
    Serial.println(inputString);
   
    //Отправляем сообщение
    LoRa.beginPacket();
    LoRa.print(inputString);
    LoRa.endPacket();
       
  }
  
  
 
  delay(50);
}
