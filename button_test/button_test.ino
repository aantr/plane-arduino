const int button_pin_0 = 9;
const int button_pin_1 = A4;

int get_button_state(int index){
  if (index == 0){
    return digitalRead(button_pin_0);
  } else if (index == 1){
    return analogRead(button_pin_1);
  }
  return 0;
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(button_pin_0, INPUT_PULLUP);
  pinMode(button_pin_1, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("B0: ");
  Serial.print(get_button_state(0));
  Serial.print(", B1: ");
  Serial.println(get_button_state(1));
}
