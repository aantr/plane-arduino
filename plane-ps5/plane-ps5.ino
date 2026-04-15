#include <Bluepad32.h>

#include <SPI.h>
#include <LoRa.h>

// определяем номера пинов, используемые трансивером
#define SYNC_WORD 0xF1

#define SS 5
#define RST 14
#define DIO0 2

//#define INV_HEIGHT
#define INV_SIDE
#define INV_MOTOR

bool halfMotor = false;
bool pressedHalfMotor = false;

// start gamepad

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    // dumpGamepad(ctl);

    ctl->setColorLED(255, 0, 0);

//    ctl->axisX(),        // (-511 - 512) left X Axis
//    ctl->axisY(),        // (-511 - 512) left Y axis
//    ctl->axisRX(),       // (-511 - 512) right X axis
//    ctl->axisRY(),       // (-511 - 512) right Y axis

    int currentHalfMotor = ctl->throttle();
    if (currentHalfMotor > 900) {
      if (pressedHalfMotor == false) {
        halfMotor = !halfMotor;
        if (halfMotor) {
          int led = 4;
          ctl->setPlayerLEDs(led & 0x0f);
          ctl->playDualRumble(0 /* delayedStartMs */, 1000 /* durationMs */, 0x80 /* weakMagnitude */,
                            0xA0 /* strongMagnitude */);
        } else {
          int led = 1;
          ctl->setPlayerLEDs(led & 0x0f);
        }
      }
      pressedHalfMotor = true;
    } else if (currentHalfMotor < 800) {
      pressedHalfMotor = false;
    }

    int heightValue = (int) ctl->axisRY() / 4 + 0x80;
    #ifdef INV_HEIGHT
    heightValue = -(int) ctl->axisRY() / 4 + 0x80;
    #endif
    int sideValue = (int) ctl->axisRX() / 4 + 0x80;
    #ifdef INV_SIDE
    sideValue = -(int) ctl->axisRX() / 4 + 0x80;
    #endif
    int motorValue = (int) ctl->axisY() / 2;
    #ifdef INV_SIDE
    motorValue = -(int) ctl->axisY() / 2;
    #endif

    if (halfMotor) {
      motorValue += 256;
      motorValue /= 2;
    }
    
    if (motorValue < 0x10) {
      motorValue = 0;
    }

    String inputString = "<";
    inputString += byteToHex(min(255, max(0, heightValue)));
    inputString += ",";
    inputString += byteToHex(min(255, max(0, sideValue)));
    inputString += ",";
    inputString += byteToHex(min(255, max(0, motorValue)));
    inputString += ">";
    
    Serial.print("Sending - ");
    Serial.println(inputString);
    
    //Отправляем сообщение
    LoRa.beginPacket();
    LoRa.print(inputString);
    LoRa.endPacket();
  
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
                break;
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
    Serial.println("No controller");
}


// end gamepad

String byteToHex(byte b) {
  char hex[3];
  sprintf(hex, "%02x", b);
  return String(hex);
}

void setup() {
  Serial.begin(115200);

  // Lora setup
  Serial.println("LoRa Sender");
 
  //настраиваем трансивер
  LoRa.setPins(SS, RST, DIO0);
  
  //замените LoRa.begin(---E-) частотой, которую вы собираетесь использовать 
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(126E3);
  LoRa.setCodingRate4(8);
  LoRa.setSyncWord(SYNC_WORD);
  
  Serial.println("LoRa Initializing OK!");

  // gamepad setup 

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  
}

void loop() {

  bool dataUpdated = BP32.update();
  processControllers();

  delay(35);
  
}
