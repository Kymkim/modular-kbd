#include <Arduino.h>
#include <BleKeyboard.h>

#define ROWS 2
#define COLS 2

int ROW[] = {15,2};
int COL[] = {0,4};

//Stores The Raw Report of the Keyboard
uint8_t keyboard_report[ROWS][COLS];

//Stores The Mapping of Each of The Key. Mapping is based of the ASCII value of that key.
//Mappings are in the 2D Space - it's shifted version is 32 bits more than the original
//Thus to get shifted version of M which is 0x4D we can add it by 32 which would be 0x6D
//
// Potential Improvements: 
// It may be possible to make a firmware by just altering this variable to the mappings are reconfigurable. 
// It may be possible to use more than 2 dimension of the array to allocate for special "shift" keys found on SFF keyboards
uint8_t keyboard_mapping[ROWS][COLS] = {
  {0x4D,0x69},
  {0x4B,0x55},
};

//Temporary variable to store old and new keyoard report.
//This way we only change the keyboard state when there is a change in the raw data.
//This allows us to hold keys and press multiple at the same time
//Rather than sending individual keystroke each press
uint8_t old_report;
uint8_t new_report;

BleKeyboard bleKeyboard("ModularKeyboard_MainBoard");

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE");
  bleKeyboard.begin();
  for (int i=0; i<COLS; i++){
    pinMode(ROW[i], INPUT);
    pinMode(COL[i], OUTPUT);
  }
}

void loop() {
  if(bleKeyboard.isConnected()){
    // Reset keyboard report
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
          keyboard_report[i][j] = 0x0;
      }
    }
    //Read keyboard report and send
    for(int i=0; i < COLS; i++){
      //Write to the column
      digitalWrite(COL[i], HIGH);
      for(int j=0; j < ROWS; j++){
        //Read row input
        old_report = keyboard_report[i][j]; //See line 25
        new_report = digitalRead(ROW[j]);
        if(old_report!=new_report){
          if(new_report){
            bleKeyboard.press(keyboard_mapping[i][j]);
          }else{
            bleKeyboard.release(keyboard_mapping[i][j]);
            bleKeyboard.release(keyboard_mapping[i][j] + 32); //Shifted version
          }
          keyboard_report[i][j] = new_report; 
        }
      }
    }
  }
}
