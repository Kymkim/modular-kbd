#include <Arduino.h>
#include <BleKeyboard.h>

#define ROWS 2
#define COLS 2

BleKeyboard bleKeyboard("ModularKeyboard_MainBoard");

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleKeyboard.begin();
}

void loop() {
  if(bleKeyboard.isConnected()){
    for(int i=0; i < COLS; i++){
      //TURN ON ROW
      for(int j=0; j < ROWS; j++){
        //READ EACH ROW
        //ENABLE KEY IN THAT PIN
      }
    }
  }
}
