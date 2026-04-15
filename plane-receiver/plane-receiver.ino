#include <Servo.h>
#include <SPI.h>
#include <LoRa.h>

#define CONNECTION_MSG_TIMEOUT 300
long long connection_timer = millis();
int lastRssi = 0;

#define SYNC_WORD 0xF1

// pins - height, - side, - motor
const int PINS[] = {3, 5, 6, 9};

#define MIN_HEIGHT_DEGREE 45
#define MAX_HEIGHT_DEGREE 180 - 45
#define MIN_SIDE_DEGREE 45
#define MAX_SIDE_DEGREE 180 - 45

#define FAIL_SAFE_TIMEOUT 1000
long long lastCorrectTime = millis();

bool calibrate = false;

// calibration
#define ESC_PIN PINS[2]
#define MAX_PULSE 1500  // Maximum throttle signal (µs)
#define MIN_PULSE 1100  // Minimum throttle signal (µs)

Servo myESC;

void setup_calibrate() {
  Serial.begin(9600);
  myESC.attach(ESC_PIN, MIN_PULSE, MAX_PULSE); // Attach with defined limits

  Serial.println("=== ESC Calibration Start ===");
  Serial.println("Disconnect battery from ESC now.");
  delay(2000);

  // Step 1: Send max throttle signal
  Serial.println("Sending MAX throttle signal...");
  myESC.writeMicroseconds(MAX_PULSE);
  delay(3000); // Wait for user to connect battery

  // --- CRITICAL STEP: CONNECT THE LIPO BATTERY TO THE ESC NOW ---
  Serial.println(">>> CONNECT THE LIPO BATTERY TO THE ESC NOW <<<");
  delay(5000); // Give time for ESC to read max signal and beep

  // Step 2: Send min throttle signal to set low endpoint
  Serial.println("Sending MIN throttle signal...");
  myESC.writeMicroseconds(MIN_PULSE);
  delay(3000); // Wait for final confirmation beeps

  Serial.println("=== Calibration Complete! ===");
  Serial.println("You can now use the Serial Monitor to test.");
  Serial.println("Enter a value between 0 and 100.");
}

void loop_calibrate() {
  if (Serial.available() > 0) {
    int throttlePercent = Serial.parseInt();
    throttlePercent = constrain(throttlePercent, 0, 100); // Clamp value

    int pulseWidth = map(throttlePercent, 0, 100, MIN_PULSE, MAX_PULSE);
    myESC.writeMicroseconds(pulseWidth);

    Serial.print("Throttle set to: ");
    Serial.print(throttlePercent);
    Serial.print("% (");
    Serial.print(pulseWidth);
    Serial.println("µs)");
  }
  delay(10);
}

int sign(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

class MyServo {
  public:
  Servo servo;
  int pin = 0;
  int bound[2] = {0, 255};

  int current = bound[0];
  int target = current;
  unsigned long target_timer = millis();
  unsigned long target_timeout = 1000 / 200;
  bool degrees = true;

  MyServo(int pin_, int bound_0, int bound_1, int target_timeout_, bool degrees_) {
    pin = pin_;
    bound[0] = bound_0;
    bound[1] = bound_1;
    target_timeout = target_timeout_;
    degrees = degrees_;
  } 

  void init(int value) {
    servo.attach(pin);
    set_value(value);
    set_target(value);
    current = target;
  }

  void update() {

    if (millis() - target_timer >= target_timeout){
      int count = (millis() - target_timer) / target_timeout;
      target_timer += target_timeout * count;
      
      if (sign(target - current)){
        for (int i=0;i<count;i++){
          current += sign(target - current);
        }
        set_value(current);
      }
    }

  }

  void set_value(int value) {
    value = min(255, max(0, value));
    int deg = (long long)(value) * (bound[1] - bound[0]) / 256 + bound[0];
    if (degrees) {
      servo.write(deg);
    } else {
      Serial.println(deg);
      servo.writeMicroseconds(deg);
    }
  }

  void set_target(int value) {
    value = min(255, max(0, value));
    target = value;
  }

  
};

// <0x,0x,0x>

#define PACKET_SIZE 10
#define PACKET_START_SYMBOL '<'
#define PACKET_END_SYMBOL '>'

struct Packet {
  bool damaged = false;
  int values[3];
  
  Packet () {
    for (int i = 0; i < 3; i++) {
      values[i] = 0;
    }
  }
  Packet (String s) {
    if (s.length() != 10 || s[0] != '<' || s[s.length() - 1] != '>') {
      damaged = true;
    } else if (s[3] != ',' || s[6] != ',') {
      damaged = true;
    } else {
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
          char c = s[1 + i * 3 + j];
          if ('0' <= c && c <= '9' || 'a' <= c && c <= 'f') {
            // pass
          } else {
            damaged = true;
          }
        }
        values[i] = strtol(s.substring(1 + i * 3, 1 + i * 3 + 2).c_str(), NULL, 16);
        values[i] = max(0, min(255, values[i]));
      }
    }
  }

  String getString() {
    return String("<") + 
    String(values[0], HEX) + String(",") + 
    String(values[1], HEX) + String(",") + 
    String(values[2], HEX) + 
    String(">");
  }
};

// servo setup
MyServo myservo_height(PINS[0], MIN_HEIGHT_DEGREE, MAX_HEIGHT_DEGREE, 1000 / 1000, true);
MyServo myservo_side(PINS[1], MIN_SIDE_DEGREE, MAX_SIDE_DEGREE, 1000 / 1000, true);
MyServo myservo_motor(PINS[2], MIN_PULSE, MAX_PULSE, 1000 / 1000, false);

void setup() {
  // Serial setup 
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  // Lora setup 
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(126E3);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(SYNC_WORD);

  // Servos setup 
  myservo_height.init(128);
  myservo_side.init(128);
  myservo_motor.init(0);

  if (calibrate) {
    setup_calibrate();
  }

}

void loop() {

  if (calibrate) {
    loop_calibrate();
    return;
  }
  
  // update all
  
  myservo_height.update();
  myservo_side.update();
  myservo_motor.update();

  if (millis() - lastCorrectTime > FAIL_SAFE_TIMEOUT) { // failsafe
     myservo_height.set_target(0x80);
     myservo_side.set_target(0x80);
     myservo_motor.set_target(0);
  }

  if (millis() - connection_timer > CONNECTION_MSG_TIMEOUT) { // connection message
     String message = "<";
     message += lastRssi;
     message += ">";
     // todo: send the message 
  }

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  String packetString = "";
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    Packet lastCorrectPacket(packetString);
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      packetString += c;
      Serial.print(c);
      if (c == PACKET_END_SYMBOL) {
        Packet receivedPacket = Packet(packetString.substring(packetString.length() - PACKET_SIZE, packetString.length()));
        if (!receivedPacket.damaged) {
          lastCorrectPacket = receivedPacket;
        }
      }
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    lastRssi = LoRa.packetRssi();

    if (!lastCorrectPacket.damaged) {
      Serial.print("packet: ");
      Serial.print(lastCorrectPacket.values[0]);
      Serial.print(lastCorrectPacket.values[1]);
      Serial.print(lastCorrectPacket.values[2]);
      Serial.println(packetString);

      myservo_height.set_target(lastCorrectPacket.values[0]);
      myservo_side.set_target(lastCorrectPacket.values[1]);
      myservo_motor.set_target(lastCorrectPacket.values[2]);

      lastCorrectTime = millis();

    } else {
      Serial.println("damaged packet!");
    }
  }
  delay(10);
}
