#define DEBUG 1
// #define DEBUG 0

#include "RF24.h"
#include <EEPROM.h>
#include <Servo.h>

// receiver setup

Servo hight_servo, side_servo, motor_servo;
const int HIGHT_SERVO_PIN = 4, SIDE_SERVO_PIN = 5, MOTOR_SERVO_PIN = 9;
int hight_servo_bound[2] = {0, 115}, side_servo_bound[2] = {35, 145};

// int motor_microseconds_bound[2] = {1100, 1500}; // [1100, 1500]
int motor_microseconds_bound[2] = {1100, 1300}; // [1100, 1500], half of power

void set_hight_value(int value){ // [-100, 100]
  value = min(100, max(-100, value));
  int deg = (value + 100) * (hight_servo_bound[1] - hight_servo_bound[0]) / 200 + hight_servo_bound[0];
  hight_servo.write(deg);
}

void set_side_value(int value){ // [-100, 100]
  value = min(100, max(-100, value));
  int deg = (value + 100) * (side_servo_bound[1] - side_servo_bound[0]) / 200 + side_servo_bound[0];
  side_servo.write(deg);
}

int hight_current = 0, side_current = 0;
int hight_target = hight_current, side_target = side_current;
unsigned long target_hight_timer = 0;
unsigned long target_side_timer = 0;

unsigned long target_hight_timeout = 1000 / 200;
unsigned long target_side_timeout = 1000 / 500;

int sign(int v){
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

void update_servo_target(){
  if (millis() - target_hight_timer >= target_hight_timeout){

    int count = (millis() - target_hight_timer) / target_hight_timeout;
    target_hight_timer += target_hight_timeout * count;
    
    if (sign(hight_target - hight_current)){
      for (int i=0;i<count;i++){
        hight_current += sign(hight_target - hight_current);
      }
      set_hight_value(hight_current);
    }
  }

  if (millis() - target_side_timer >= target_side_timeout){

    int count = (millis() - target_side_timer) / target_side_timeout;
    target_side_timer += target_side_timeout * count;
    
    if (sign(side_target - side_current)){
      for (int i=0;i<count;i++){
        side_current += sign(side_target - side_current);
      }
      set_side_value(side_current);
    }
  }
}

void set_hight_target(int value){
  value = min(100, max(-100, value));
  hight_target = value;
}

void set_side_target(int value){
  value = min(100, max(-100, value));
  side_target = value;
}

void set_motor_value(int value){
  value = min(100, max(0, value));
  int microSeconds = map(value, 0, 100, motor_microseconds_bound[0], motor_microseconds_bound[1]);
  motor_servo.writeMicroseconds(microSeconds);
}

void init_receiver(){
  hight_servo.attach(HIGHT_SERVO_PIN);
  side_servo.attach(SIDE_SERVO_PIN);
  set_hight_value(hight_current);
  set_side_value(side_current);

  motor_servo.attach(MOTOR_SERVO_PIN);
  motor_servo.writeMicroseconds(1100);
  delay(1000);

}

// transmitter setup

#include <Adafruit_NeoPixel.h>
const int led_pin = 6;
const int num_leds = 3;
Adafruit_NeoPixel leds(num_leds, led_pin, NEO_GRB + NEO_KHZ800);

const int stick_pin[4] = {A3, A2, A1, A0};
const int stick_bound[4][3] = { {115, 650, 1023 - 50}, {85, 540, 1023 - 50}, {75, 580, 1023 - 50}, {75, 565, 1023 - 50} };
const int button_pin_0 = 8;
const int button_pin_1 = A4;

int get_button_state(int index){
  if (index == 0){
    return digitalRead(button_pin_0) ^ 1;
  } else if (index == 1){
    return (analogRead(button_pin_1) >= 75) ^ 1;
  }
  return 0;
}

int get_bounded_stick_value(int bound[3], int val){
  val = max(min(val, bound[2]), bound[0]);
  if (val >= bound[1]) return map(val, bound[1], bound[2], 0, 100);
  return map(val, bound[0], bound[1], -100, 0);
}

int get_left_horizontal(){
  return get_bounded_stick_value(stick_bound[0], analogRead(stick_pin[0]));
}

int get_left_vertical(){
  return get_bounded_stick_value(stick_bound[1], analogRead(stick_pin[1]));
}

int get_right_horizontal(){
  return get_bounded_stick_value(stick_bound[2], analogRead(stick_pin[2]));
}

int get_right_vertical(){
  return get_bounded_stick_value(stick_bound[3], analogRead(stick_pin[3]));
}

void init_transmitter(){
  for (int i=0;i<4;i++){
    pinMode(stick_pin[i], INPUT_PULLUP);
  }
  pinMode(button_pin_0, INPUT_PULLUP);
  pinMode(button_pin_1, INPUT_PULLUP);

  leds.begin();
  leds.clear();
  leds.show();

  update_leds();

}

const int RADIO_PIN_RECEIVER[2] = {7, 8}, RADIO_PIN_TRANSMITTER[2] = {9, 10};

RF24 radio;
byte radio_address[][6] = {"1Node", "2Node"};
bool radio_receiver = 0;

const int size_data = 12;

struct Packet{
  bool query = 0;
  byte type = 0;
  byte data[size_data];
  Packet(){
    memset(data, 0, sizeof(data));
  }
};

struct RadioManager{

  int connected = 0;
  unsigned long transmission_time = 0;
  long long received = 0;
  long long failed = 0;
  long long transmitted = 0;

  void init(){

    if (radio_receiver){
      if (!radio.begin(RADIO_PIN_RECEIVER[0], RADIO_PIN_RECEIVER[1])) {if (DEBUG) Serial.println(F("[Receiver] radio hardware is not responding"));while (1) {}}
    }else{
      if (!radio.begin(RADIO_PIN_TRANSMITTER[0], RADIO_PIN_TRANSMITTER[1])) {if (DEBUG) Serial.println(F("[Transmitter] radio hardware is not responding"));while (1) {}}
    }

    radio.setAutoAck(1);
    radio.setRetries(0, 15);
    // radio.enableAckPayload();

    if (sizeof(Packet) > 32){
      if (DEBUG) Serial.print("Too big packet size: ");
      if (DEBUG) Serial.println(sizeof(Packet));
      while (1){}
    }
    
    radio.setPayloadSize(sizeof(Packet));

    radio.openWritingPipe(radio_address[radio_receiver]);
    radio.openReadingPipe(1, radio_address[!radio_receiver]);
    radio.setPALevel(RF24_PA_LOW);
    
    radio.setChannel(0x6B);
    radio.setDataRate(RF24_1MBPS);

    radio.powerUp();

    if (radio_receiver) radio.startListening();
    else radio.stopListening();
    
  }

  Packet get_hight_packet(int val){ // id 0
    val = min(100, max(-100, val));
    Packet packet;
    packet.type = 0;
    packet.data[0] = val + 100;
    return packet;
  }

  Packet get_side_packet(int val){ // id 1
    val = min(100, max(-100, val));
    Packet packet;
    packet.type = 1;
    packet.data[0] = val + 100;
    return packet;
  }

  Packet get_motor_value_packet(int val){ // id 2
    val = min(100, max(0, val));
    Packet packet;
    packet.type = 2;
    packet.data[0] = val;
    return packet;
  }

  Packet get_full_packet(int height, int side, int motor){ // id 10
    height = min(100, max(-100, height));
    side = min(100, max(-100, side));
    motor = min(100, max(0, motor));
    Packet packet;
    packet.type = 10;
    packet.data[0] = height + 100;
    packet.data[1] = side + 100;
    packet.data[2] = motor;
    return packet;
  }

  Packet update_transmitter(Packet packet){
    int success;
    update_transmitter(packet, success);
  }

  Packet update_transmitter(Packet packet, int &success){
    
      unsigned long start_timer = micros();    
      bool report = radio.write(&packet, sizeof(packet));   
  
      if (report) {
        
        if (packet.query){
          radio.startListening();

          unsigned long time_start_reading_query = millis();
          int failed = 0;

          byte pipe;
          while (!radio.available(&pipe)){
            if (millis() - time_start_reading_query > 150){ // 150 ms answer timeout
              failed = 1;
              break;
            }
            delay(10);
          }

          if (!failed){

            // receive
            int bytes = radio.getPayloadSize(); 
            radio.read(&packet, bytes);
    
            if (DEBUG) Serial.print(F("Received ")); 
            if (DEBUG) Serial.print(bytes);          
            if (DEBUG) Serial.print(F(" bytes on pipe "));
            if (DEBUG) Serial.print(pipe);  
            if (DEBUG) Serial.print(" in ");
            if (DEBUG) Serial.print(millis() - time_start_reading_query);         
              if (DEBUG) Serial.println(" ms");                   
          
            received ++ ;

            delay(20);
            
          } else{
            if (DEBUG) Serial.println(F("Readind query failed or timed out")); 
          }

          radio.stopListening();

          if (!failed){
            success = 1;
            return packet;
          }

          success = 0;
          return Packet();
          
        } else{
          unsigned long end_timer = micros(); 
          if (DEBUG) Serial.print(F("Transmission successful! "));      
          if (DEBUG) Serial.print(F("Time to transmit = "));
          if (DEBUG) Serial.print(end_timer - start_timer);    
          if (DEBUG) Serial.println(F(" us."));               
          transmitted++;
          transmission_time = end_timer - start_timer;
          success = 1;
          return Packet();
        }

        
      } else {
        if (DEBUG) Serial.println(F("Transmission failed or timed out")); 
        failed++;
      }
      success = 0;
      return Packet();
  }

  void update_receiver(){
    Packet packet;
    
    byte pipe;
    if (radio.available(&pipe)) {
      int bytes = radio.getPayloadSize(); 
      radio.read(&packet, bytes);

      if (packet.query){

        delay(40);

        radio.stopListening();

        Packet answer = get_answer_packet(packet);
      
        unsigned long start_timer = micros();    
        bool report = radio.write(&answer, sizeof(Packet));   
        unsigned long end_timer = micros(); 
    
        if (report) {
          if (DEBUG) Serial.print(F("Answer successful! "));      
          if (DEBUG) Serial.print(F("Time to answer = "));
          if (DEBUG) Serial.print(end_timer - start_timer);    
          if (DEBUG) Serial.print(F(" us."));              
          transmitted++;
        } else{
          if (DEBUG) Serial.println(F("Answer failed or timed out")); 
          failed++;
        }

        radio.startListening();
        
      } else {
        if (DEBUG) Serial.print(F("Received ")); 
        if (DEBUG) Serial.print(bytes);          
        if (DEBUG) Serial.print(F(" bytes on pipe "));
        if (DEBUG) Serial.print(pipe);     
        if (DEBUG) Serial.println(".");            
        received ++ ;
        
        // process the data
        process_received_data(packet);
      } 
    }

    delay(10);
  }

  Packet get_answer_packet(Packet input){
    Packet answer;
    if (input.type == 255){
      answer.type = 255;
      connection_confirmed();
    }
    return answer;
  }

  void process_received_data(Packet packet){
    connection_confirmed();

    if (packet.type == 0){
      set_hight_target((int) packet.data[0] - 100);
      if (DEBUG) Serial.print("Update hight value ");
      if (DEBUG) Serial.println(packet.data[0]);
    }else if (packet.type == 1){
      set_side_target((int) packet.data[0] - 100);
      if (DEBUG) Serial.print("Update side value ");
      if (DEBUG) Serial.println(packet.data[0]);
    } else if (packet.type == 2){
      set_motor_value((int) packet.data[0]);
      if (DEBUG) Serial.print("Update motor value ");
      if (DEBUG) Serial.println(packet.data[0]);
    } else if (packet.type == 10){
      set_hight_target((int) packet.data[0] - 100);
      set_side_target((int) packet.data[1] - 100);
      set_motor_value((int) packet.data[2]);

    }
  }
  
};
// common

RadioManager rmanager;

// transmitter update

unsigned long check_connection_timer = 0;
unsigned long check_connection_timeout = 300;
unsigned long check_connection_timeout_disconnected = 100;

unsigned long button_update_timer = 0;
unsigned long button_update_timeout = 65;

unsigned long update_state_timer = 0;
unsigned long update_state_timeout = 80;

int prev_button_state1 = 0;
int fixed_motor = 0;
int fixed_motor_value = 0;
int fixed_side_value = 0;
int fixed_height_value = 0;

void debug_state(){
  Serial.print("LH: ");
  Serial.print(get_left_horizontal());
  Serial.print(", ");
  Serial.print("LV: ");
  Serial.print(get_left_vertical());
  Serial.print(", ");
  Serial.print("RH: ");
  Serial.print(get_right_horizontal());
  Serial.print(", ");
  Serial.print("RV: ");
  Serial.print(get_right_vertical());
  Serial.print(", ");
  Serial.print("B0: ");
  Serial.print(get_button_state(0));
  Serial.print(", ");
  Serial.print("B1: ");
  Serial.print(get_button_state(1));
  Serial.print(", ");
  Serial.println(analogRead(button_pin_1));
}

void write_current_state(){
  // if (DEBUG) Serial.println("Sent current state");
  int left_vertical = get_left_vertical();
  int right_horizontal = get_right_horizontal();
  int right_vertical = get_right_vertical();

  if (fixed_motor){
    right_vertical = fixed_height_value;
    right_horizontal = fixed_side_value;
    left_vertical = fixed_motor_value;
  }

  if (left_vertical < 10) left_vertical = 0;
  if (abs(right_horizontal) < 10) right_horizontal = 0;
  if (abs(right_vertical) < 10) right_vertical = 0;

  Packet packet = rmanager.get_full_packet(-right_vertical, right_horizontal, left_vertical);
  rmanager.update_transmitter(packet);
}

void update_button1(){
  int state = get_button_state(1);
  if (state && !prev_button_state1){
    // switch fixed motor
    fixed_motor ^= 1;
    if (fixed_motor){
      fixed_height_value = get_right_vertical();
      fixed_side_value = get_right_horizontal();
      fixed_motor_value = get_left_vertical();
    }
  }
  prev_button_state1 = state;
}

void update_leds(){
  leds.clear();
  if (rmanager.connected){
    leds.setPixelColor(1, leds.Color(0, 100, 0));
    if (rmanager.transmission_time <= 1000){
      leds.setPixelColor(2, leds.Color(0, 100, 0));
    }
  } else {
    leds.setPixelColor(1, leds.Color(100, 0, 0));
  }

  if (fixed_motor){
    leds.setPixelColor(0, leds.Color(0, 0, 100));
  }

  leds.show();
}

void update_connected(){
  update_leds();
}

void update_transmitter(){

  if (DEBUG) debug_state();

  if (rmanager.connected && millis() - check_connection_timer >= check_connection_timeout || 
      !rmanager.connected && millis() - check_connection_timer >= check_connection_timeout_disconnected){
    int prev_connected = rmanager.connected;
    check_connection_timer = millis();
    int res = 1;
    Packet check_connection;
    // check_connection.query = 1;
    check_connection.query = 0;
    check_connection.type = 255;
    Packet answer = rmanager.update_transmitter(check_connection, res);
    // rmanager.connected = bool(res) && answer.type == 255;
    rmanager.connected = bool(res);
    if (rmanager.connected != prev_connected){
      update_connected();
    }
  }

  // if (rmanager.connected) { 

  if (millis() - update_state_timer >= update_state_timeout){
    update_state_timer = millis();

    write_current_state();

  }

  if (millis() - button_update_timer >= button_update_timeout){
    button_update_timer = millis();
    update_button1();
    update_leds();
  }
    
  // }
  
}

// receiver update

unsigned long receiver_connection_timer = 0;
unsigned long receiver_connection_timeout = 700;

void set_default_state(){
  set_hight_target(0);
  set_side_target(0);
  set_motor_value(0);  
}

void connection_confirmed(){
  rmanager.connected = 1;
  receiver_connection_timer = millis();
}

void update_receiver(){
  if (millis() - receiver_connection_timer >= receiver_connection_timeout){
    receiver_connection_timer = millis();
    rmanager.connected = 0;
    set_default_state();
  }
}

void setup() {

  if (DEBUG){
    Serial.begin(9600);
    while (!Serial) {}
  }
  EEPROM.get(1, radio_receiver);
  radio_receiver = bool(radio_receiver);

  if (radio_receiver){
    init_receiver();
  } else {
    init_transmitter();
  }
  
  if (DEBUG) Serial.println("Radio receiver: " + String(radio_receiver));
  rmanager.init();
  
}

void loop() {

  if (radio_receiver){
    rmanager.update_receiver();
    update_servo_target();
    update_receiver();
  } else {
    update_transmitter();
  }

  if (DEBUG && Serial.available()) {
    if (radio_receiver){ // receiver
      char c = Serial.read();
      if (c == 'H'){
         int val = Serial.parseInt();
         set_hight_target(val);
         
      } else if (c == 'S'){
        int val = Serial.parseInt();
        set_side_target(val);
      } else if (c == 'M'){
        int val = Serial.parseInt();
        set_motor_value(val);
      }
    } else { // transmitter
      char c = Serial.read();
      if (c == 'H'){
         int val = Serial.parseInt();
         rmanager.update_transmitter(rmanager.get_hight_packet(val));
      } else if (c == 'S'){
        int val = Serial.parseInt();
        rmanager.update_transmitter(rmanager.get_side_packet(val));
      } else if (c == 'M'){
        int val = Serial.parseInt();
        rmanager.update_transmitter(rmanager.get_motor_value_packet(val));
      }
    }
    
  }

}
