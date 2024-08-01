#include <Servo.h>
#include <SoftwareSerial.h>

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

// transmitter setup

#define REVERSE_HIGHT
#define REVERSE_SIDE

SoftwareSerial HC12(9, 10);
const int set_pin = 12;

#include <Adafruit_NeoPixel.h>
const int led_pin = 6;
const int num_leds = 3;
Adafruit_NeoPixel leds(num_leds, led_pin, NEO_GRB + NEO_KHZ800);

const int stick_pin[4] = {A3, A2, A1, A0};
int stick_bound[4][3] = { {115, 650, 1023 - 50}, {85, 540, 1023 - 50}, {75, 580, 1023 - 50}, {75, 565, 1023 - 50} };
const int button_pin_0 = 8;
const int button_pin_1 = A4;
const int button_pin_2 = 0;
const int button_pin_3 = 13;
const int button_pin_4 = 11;

const int motor_restrict = 80; // [0, 100]

Timer transmit_timer;
const int transmitter_delay = 5;
const int transmitter_timeout = 150;
int transmitted_data = -1;
int received_data = -1;
int button_0_state = 0;
int button_4_state = 0;
int success = 1;
int motor_lock = 0;

int get_button_state(int index) {
  if (index == 0){
    return digitalRead(button_pin_0) ^ 1;
  } else if (index == 1) {
    return analogRead(button_pin_1) < 512;
  } else if (index == 2) {
    return 0;
  } else if (index == 3) {
    return digitalRead(button_pin_3) ^ 1;
  } else if (index == 4) {
    return digitalRead(button_pin_4) ^ 1;
  }
  return 0;
}

int get_bounded_stick_value(int bound[3], int val) {
  val = max(min(val, bound[2]), bound[0]);
  if (val >= bound[1]) return map(val, bound[1], bound[2], 0, 100);
  return map(val, bound[0], bound[1], -100, 0);
}

int get_left_horizontal() {
  return get_bounded_stick_value(stick_bound[0], analogRead(stick_pin[0]));
}

int get_left_vertical() {
  return get_bounded_stick_value(stick_bound[1], analogRead(stick_pin[1]));
}

int get_right_horizontal() {
#ifdef REVERSE_SIDE
  return -1 * get_bounded_stick_value(stick_bound[2], analogRead(stick_pin[2]));
#else
  return get_bounded_stick_value(stick_bound[2], analogRead(stick_pin[2]));
#endif
}

int get_right_vertical() {
#ifdef REVERSE_HIGHT
  return -1 * get_bounded_stick_value(stick_bound[3], analogRead(stick_pin[3]));
#else
  return get_bounded_stick_value(stick_bound[3], analogRead(stick_pin[3]));
#endif
}

int get_motor_value() {
  int result = get_left_vertical();
  if (motor_lock != -1) {
    result = motor_lock;
  };
  if (result > motor_restrict) {
    result = motor_restrict;
  }
  return result;
}

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
  Serial.print(analogRead(button_pin_1));

  Serial.print(", ");
  Serial.print("B3: ");
  Serial.print(get_button_state(3));

  Serial.print(", ");
  Serial.print("B4: ");
  Serial.print(get_button_state(4));
  Serial.println();
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

void reset_neutral_stick() {
  for (int i = 0; i < 4; i++) {
    stick_bound[i][1] = analogRead(stick_pin[i]);
  }
}

void init_transmitter(){
  for (int i=0;i<4;i++){
    pinMode(stick_pin[i], INPUT_PULLUP);
  }
  randomSeed(74);
  pinMode(button_pin_0, INPUT_PULLUP);
  pinMode(button_pin_1, INPUT_PULLUP);
  // pinMode(button_pin_3, INPUT_PULLUP);
  pinMode(button_pin_4, INPUT_PULLUP);

  leds.begin();
  leds.setBrightness(200);

  leds.clear();
  leds.show();

  HC12.begin(57600);
  pinMode(set_pin, OUTPUT);

  digitalWrite(set_pin, LOW);
  delay(100);

  // AT+C019 - OK+C001
  HC12.write("AT+C001");
  delay(100);
  if (readString(HC12) != String("OK+C001")) {
    Serial.println("Problem while setting hc12 channel");
    while (1) {}
  }
  HC12.write("AT+B57600");
  delay(100);
  if (readString(HC12) != String("OK+B57600")) {
    Serial.println("Problem while setting hc12 baud rate");
    while (1) {}
  }
  digitalWrite(set_pin, HIGH);
  delay(100);

  reset_neutral_stick();

}

void transmit_data() {
  int control_sum = 0;
  uint8_t header = 3 + (1 << 3);
  uint8_t data;
  HC12.write(header);
  control_sum = (control_sum + header) % 251;
  delay(transmitter_delay);

  data = (1 << 7) + (get_right_horizontal() + 100) / 2;
  HC12.write(data);
  control_sum = (control_sum + data) % 251;
  delay(transmitter_delay);

  data = (1 << 7) + (get_right_vertical() + 100) / 2;
  HC12.write(data);
  control_sum = (control_sum + data) % 251;
  delay(transmitter_delay);

  int left_vertical = get_motor_value();
  data = (1 << 7) + (left_vertical + 100) / 2;
  HC12.write(data);
  control_sum = (control_sum + data) % 251;
  delay(transmitter_delay);

  HC12.write(control_sum);
  delay(transmitter_delay);
  transmitted_data = control_sum;

}

void update_transmitter() {

  while (HC12.available()) {   
    int read = HC12.read();
    received_data = (uint8_t) read;
    Serial.println(read);
  }
  if (get_button_state(0)) {
    if (button_0_state == 0) {
      if (motor_lock == -1) {
        motor_lock = get_motor_value();
      } else {
        motor_lock = -1;
      }
    }
    button_0_state = 1;
  } else {
    button_0_state = 0;
  }
  if (get_button_state(4)) {
    if (button_4_state == 0) {
      reset_neutral_stick();
    }
    button_4_state = 1;
  } else {
    button_4_state = 0;
  }
  transmit_timer.update();
  if (transmit_timer.expired(transmitter_timeout)) {
    if (received_data != -1 && received_data == transmitted_data) {
      success = 2;
    } else {
      success = 1;
    }
    received_data = -1;
    transmit_data();
  }
  leds.setPixelColor(2, leds.Color(0, 255 * button_4_state, 0));
  leds.setPixelColor(1, leds.Color(0, 0, 255 * (motor_lock != -1)));
  if (success == 0) {
    leds.setPixelColor(0, leds.Color(0, 0, 255));
  } else if (success == 1) {
    leds.setPixelColor(0, leds.Color(255, 0, 0));
  } else {
    leds.setPixelColor(0, leds.Color(0, 255, 0));
  }
  
  leds.show();
  delay(10);
}

void setup() {

  Serial.begin(9600);
  while (!Serial) {}
  
  init_transmitter();
  Serial.println("Init");
  
}

void loop() {
  update_transmitter();
}