#include <SoftwareSerial.h>
#include <Servo.h>
 
SoftwareSerial HC12(7, 8); // HC-12 TX Pin, HC-12 RX Pin plane
const int set_pin = 9;

// sound pin
const int sound_pin = 10;

// motor
Servo motor;
const int motor_pin = 11;
const int motor_max = 2300;
const int motor_min = 1000;
const int motor_restrict = 100; // [0, 100]
const bool need_calibration = true;

void set_motor_value(int value){ // [0, 100]
  value = min(motor_restrict, max(0, value));
  int val = value * (motor_max - motor_min) / 100 + motor_min;
  motor.writeMicroseconds(val);
}

// servos
int hight_servo_bound[2] = {0, 115}, side_servo_bound[2] = {35, 145};
Servo hight_servo, side_servo;
const int side_servo_pin = 6;
const int hight_servo_pin = 5;


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

String readString(SoftwareSerial &ss) {
  String res;
  while (ss.available()) {
    delay(5);
    auto read = ss.read();
    if (read == 13) {
      continue;
    } else if (read == 10) {
      break;
    }
    res = res + (char) read;
  }
  return res;
}

class Timer {
public:
  unsigned long current_time, timer;
  Timer() {
    current_time = millis();
    timer = 0;
  }

  void update() {
    auto time = millis();
    timer += time - current_time;
    current_time = time;
  }

  void reset() {
    timer = 0;
  }

  bool expired(unsigned long time) {
    if (time == 0) {
      return false;
    }
    if (timer >= time) {
      timer -= time;
      if (timer >= time) {
        timer %= time;
      }
      return true;
    }
    return false;
  }
};

class SmoothValue {
public:
  long long value;
  long long target;
  long long speed; // per second
  unsigned long prev_time;
  long long prev_value;

  SmoothValue(long long value, long long speed): value(value), speed(speed) {
    target = value;
    prev_value = value;
    prev_time = millis();
  }

  long long get() {
    return value;
  }

  void setSpeed(long long v) {
    speed = v;
  }

  void set(long long v) {
    target = v;
    prev_time = millis();
    prev_value = value;
  }

  void update() {
    if (value != target && target != prev_value) {
      // update value
      auto time = millis();
      long long total = (time - prev_time) * speed / 1000;
      if (speed == 0 || total >= abs(target - prev_value)) {
        value = target;
        return;
      }
      int sign = (target - prev_value) / abs(target - prev_value);
      value = prev_value + total * sign;
    }
  }
};

void sound_dot() {
  digitalWrite(sound_pin, HIGH);
  delay(100); 
  digitalWrite(sound_pin, LOW);
  delay(50);
}

void sound_dash() {
  digitalWrite(sound_pin, HIGH);
  delay(350); 
  digitalWrite(sound_pin, LOW);
  delay(50);
}

bool setup_error = false;

Timer led_timer;
int led_timer_delay = 300;
bool led_state = 0;

SmoothValue motor_value(0, 100), side_value(0, 0), hight_value(0, 0);
const int state_timer_delay = 30; // ~ 30 fps

// receiver state
int receiver_state = -1; // -1 - wait for header, 0 - wait for control sum, x > 0 - number of bytes left
const int max_bytes = 8;
uint8_t header_data = 0;
uint8_t receiver_data[max_bytes] = {};
int current_packet_size = 0;
int current_control_sum = 0;

// connection
Timer connection_timer;
const unsigned long connection_timeout = 1000;

void setup() {

  setup_error = false;
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(set_pin, OUTPUT);
  pinMode(sound_pin, OUTPUT);
  Serial.begin(9600);

  Serial.println("Init motor");

  // motor setup start
  motor.attach(motor_pin);
  if (need_calibration) {
    motor.writeMicroseconds(motor_max);
  } else {
    motor.writeMicroseconds(motor_min);
  }

  sound_dot();

  Serial.println("Init servo");

  // servo setup
  side_servo.attach(side_servo_pin);
  hight_servo.attach(hight_servo_pin);
  set_side_value(0);
  set_hight_value(0);

  Serial.println("Init hc12");

  // hc12 setup
  HC12.begin(57600);
  delay(15);
  digitalWrite(set_pin, LOW);
  delay(100);
  // AT+C019 - OK+C001
  HC12.write("AT+C001");
  delay(100);
  if (readString(HC12) != String("OK+C001")) {
    Serial.println("Problem while setting hc12 channel");
    setup_error = true;
  }
  if (setup_error) {
    led_timer_delay = 1000;
    sound_dash();
    Serial.println("Init error");
    return;
  }

  HC12.write("AT+B57600");
  delay(100);
  if (readString(HC12) != String("OK+B57600")) {
    Serial.println("Problem while setting hc12 baud rate");
    setup_error = true;
  }

  if (setup_error) {
    led_timer_delay = 1000;
    sound_dash();
    Serial.println("Init error");
    return;
  }

  digitalWrite(set_pin, HIGH);
  delay(100);

  // motor end setup
  if (need_calibration) {
    Serial.println("Wait motor");
    // motor setup end
    delay(2000);
    motor.writeMicroseconds(motor_min);
    delay(6000);
  } else {
    delay(3000);
  }
  
  sound_dot();
  sound_dot();
  Serial.println("Init ok");
  connection_timer.reset();
}

void on_packet_received(int side, int hight, int motor) {
  side = side * 2 - 100;
  hight = hight * 2 - 100;
  motor = motor * 2 - 100;
  side_value.set(side);
  hight_value.set(hight);
  motor_value.set(motor);

  // reset connection
  connection_timer.update();
  connection_timer.reset();
}

void set_default_state() {
  side_value.set(0);
  hight_value.set(0);
  motor_value.set(0);
}

void read_header(uint8_t data) {
  data = data & ((1 << 7) - 1);
  current_packet_size = data & ((1 << 3) - 1); // first 3 bits
  header_data = data >> 3; // next 4 bits
  current_control_sum = (data) % 251;
  receiver_state = current_packet_size;
}

void loop() {
  led_timer.update();
  if (led_timer.expired(led_timer_delay)) {
    led_state ^= 1;
    digitalWrite(LED_BUILTIN, led_state);
  }
  if (setup_error) {
    delay(5);
    return;
  }
  while (HC12.available()) {
    auto data = HC12.read();
    // Serial.println(data);
    bool err = false;
    uint8_t highest_bit = data >> 7 & 1;
    if (receiver_state == -1 && highest_bit == 0) {
      read_header(data);
    } else if (receiver_state == 0) {
      if (current_control_sum == data) {
        // Serial.println("Control sum is ok");
        HC12.write(data);
        delay(5);
        if (current_packet_size == 3 && header_data == 1) {
          on_packet_received(receiver_data[0], receiver_data[1], receiver_data[2]);
        }
      } else {
        // Serial.print("Control sum is incorrect, ");
        // Serial.print(current_control_sum);
        // Serial.print(", ");
        // Serial.println(data);
      }
      receiver_state = -1;
    } else if (receiver_state > 0 && highest_bit == 1) {
      data = data & ((1 << 7) - 1);
      receiver_data[current_packet_size - receiver_state] = data;
      current_control_sum = (current_control_sum + data + (1 << 7)) % 251;
      receiver_state--;
    } else {
      if (highest_bit == 0) {
        receiver_state = -1;
        read_header(data);
      } else {
        err = true;
      }
    }
    if (err) {
      receiver_state = -1;
    }
    delay(5);
  }

  // connection timer check if expired
  connection_timer.update();
  if (connection_timer.expired(connection_timeout)) {
    set_default_state();
    sound_dot();
  }

  while (Serial.available()) {
    char byte = Serial.read();
    if (byte == 'M') {
      int value = Serial.parseInt();
      motor_value.set(value);
    } else if (byte == 'S') {
      int value = Serial.parseInt();
      side_value.set(value);
    } else if (byte == 'H') {
      int value = Serial.parseInt();
      hight_value.set(value);
    }
  }

  // write servos and motor
  hight_value.update();
  side_value.update();
  motor_value.update();

  set_hight_value(hight_value.get());
  set_side_value(side_value.get());
  set_motor_value(motor_value.get());

  delay(state_timer_delay);
}

/* 

Packet description

header - data bytes - control sum by modulo 251

first byte is from range [0, 128) (7 bit)
first three lowest bits sets the number of bytes to be read
next four bits is header data

then read data bytes, each from range [128, 256)
then should be control sum of data bytes and header

I send values in range [0, 100]

*/