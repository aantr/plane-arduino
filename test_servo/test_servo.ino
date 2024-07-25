#include <SoftwareSerial.h>
#include <Servo.h>
 
// SoftwareSerial HC12(9, 10); // HC-12 TX Pin, HC-12 RX Pin
SoftwareSerial HC12(3, 4); // HC-12 TX Pin, HC-12 RX Pin plane
const int set_pin = 2;
const int sound_pin = 8;

// motor
Servo motor;
const int motor_pin = 11;
const int motor_max = 2300;
const int motor_min = 1000;
const int motor_restrict = 30; // [0, 100]
const bool need_calibration = true;

void set_motor_value(int value){ // [0, 100]
  value = min(motor_restrict, max(0, value));
  int val = value * (motor_max - motor_min) / 100 + motor_min;
  motor.writeMicroseconds(val);
}

// servos
int hight_servo_bound[2] = {0, 115}, side_servo_bound[2] = {35, 145};
Servo hight_servo, side_servo;
const int side_servo_pin = 9;
const int hight_servo_pin = 10;


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
    if (value != target) {
      // update value
      auto time = millis();
      long long total = (time - prev_time) * speed / 1000;
      if (total >= abs(target - prev_value)) {
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
 

void setup() {

  // motor setup start
  motor.attach(motor_pin);
  if (need_calibration) {
    motor.writeMicroseconds(motor_max);
  } else {
    motor.writeMicroseconds(motor_min);
  }

  Serial.begin(9600);
  pinMode(sound_pin, OUTPUT);
  sound_dot();


  side_servo.attach(side_servo_pin);
  hight_servo.attach(hight_servo_pin);
  set_side_value(0);
  set_hight_value(0);

  // motor end setup
  if (need_calibration) {
    delay(2000);
    motor.writeMicroseconds(motor_min);
    delay(6000);
  } else {
    delay(3000);
  }

  sound_dot();
  sound_dot();

}

SmoothValue motor_value(0, 50), side_value(0, 300), hight_value(0, 300);

void loop() {
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

  hight_value.update();
  side_value.update();
  motor_value.update();

  set_hight_value(hight_value.get());
  set_side_value(side_value.get());
  set_motor_value(motor_value.get());

  delay(15);

}
