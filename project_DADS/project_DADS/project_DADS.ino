/**********************************************************************
  Filename    : project_Dads.ino
  Product     : drone control & recovery
  Description : Part of our UNLV Fall 2024 Senior Design Competition.
  Authors     : Nathan Ramos, Adam Sheta, Quinn Frady
  Modification: 12/6/24 STABLE VERSION 3
**********************************************************************/
#include "Freenove_4WD_Car_for_Arduino.h"
#include "BluetoothOrders.h"
#include <Wire.h>
//--------------------------- bluetooth control -------
#define UPLOAD_VOL_TIME    3000
#define COMMANDS_COUNT_MAX  8
#define INTERVAL_CHAR '#'
//-------------------------- drive to line ------------
#define RETREAT_DELAY         300
#define DRIVE_TO_LINE_DELAY   20
//-------------------------- line following -----------
#define FOLLOW_LINE_DELAY       10
#define MOTOR_STARTUP_DELAY     100
#define TK_STOP_SPEED           0
#define TK_FORWARD_SPEED        (105 + tk_Vcomp)
#define TK_TURN_SPEED_LV4       (180 + tk_Vcomp)    
#define TK_TURN_SPEED_LV3       (150 + tk_Vcomp)    
#define TK_TURN_SPEED_LV2       (-140 + tk_Vcomp)
#define TK_TURN_SPEED_LV1       (-160 + tk_Vcomp)

// // MANUAL SPEEDS FOR MARK I
// #define TK_FORWARD_SPEED        120
// #define TK_TURN_SPEED_LV4       230
// #define TK_TURN_SPEED_LV3       200
// #define TK_TURN_SPEED_LV2       -200
// #define TK_TURN_SPEED_LV1       -230
//--------------------------- docking -----------------
#define PARKING_DELAY       200
#define DOCK_IN_BASE_DELAY  5
//--------------------------- debugging ---------------
#define ON            false
#define NO_BLUETOOTH  false
#define NO_IR_SENSOR  false
//-----------------------------------------------------

// bluetooth and app voltage
u32 lastUploadVoltageTime;
String inputStringBLE = "";
bool stringComplete = false;
bool ignore_BT;
char commandBT;
int parameters[COMMANDS_COUNT_MAX], parameterCount = 0;

// I2C slave code
bool ignore_I2C;
volatile byte new_command = 0; 
volatile int x = 0;
volatile int parkingID; 
int parkingSpot;
bool master_control; 
bool drive_to_line, follow_line, dock_in_base; // modes within recovery
uint8_t in_recovery, is_parked; // status flags for drone kit
int turn_dir;
int tk_Vcomp;  //define Voltage Speed Compensation

void setup() {
  commandBT = 0;
  // flags
  drive_to_line = false;
  follow_line = false;
  dock_in_base = false;
  master_control = false;
  in_recovery = false;
  is_parked = false;
  ignore_BT = false;
  ignore_I2C = false;
  // other
  tk_Vcomp = 0;
  turn_dir = 0;
  parkingSpot = 0; // DO NOT CHANGE
  parkingID = 0; // base station will assign a parking ID
  Wire.begin(4);  
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestStatus); // requesting status
  pinsSetup();
  Serial.begin(9600);
  // tk_CalculateVoltageCompensation();
  // debug mode
  if (ON) {
    parkingID = 2;
    tk_Vcomp = 0;
    if (NO_IR_SENSOR) {
      ignore_I2C = true;
      Wire.end();
      new_command = 0;
    }
    if (NO_BLUETOOTH) {
      ignore_BT = true;
      alarm(5,3);
      Serial.end();
      recoverymode();
    }
  }
}

void loop() {
  if (millis() - lastUploadVoltageTime > UPLOAD_VOL_TIME) {
    upLoadVoltageToApp();
    lastUploadVoltageTime = millis();
  }
  // listen to base station first
  if (new_command) {
    process_command(x);
  } else { // listen to bluetooth control
    switch (commandBT) {
      case ACTION_CAR_MODE:
        if (parameters[1] == 3) {
          recoverymode();
          in_recovery = false;
        }
        break;
      case ACTION_MOVE:
        if (parameterCount == 2) {
          motorRun(parameters[1], parameters[2]);
        }
        break;
      case ACTION_BUZZER:
        if (parameterCount == 1) {
          setBuzzer(parameters[1]);
        }
        break;
      default:
        resetCarAction();
        break;
    }
  }
}

void recoverymode() {
  bool black_line_found = false;
  in_recovery = true;
  follow_line = false;
  dock_in_base = false;
  is_parked = false;
  drive_to_line = (ignore_I2C) ? true : false;
  u8 trackVal = getTrackingSensorVal();
  u8 prevTrackVal;
  while (true) {
    // listen to base station first
    if(new_command){
      process_command(x);
      // Serial.println("New Command Processed");
    }
    // listening to bluetooth control
    serialEvent();
    if ((!ignore_BT) && (commandBT == ACTION_CAR_MODE && parameters[1] == 0)) {
      follow_line = false;
      drive_to_line = false;
      dock_in_base = false;
      resetCarAction();
      return;
    }
    prevTrackVal = trackVal;
    trackVal = getTrackingSensorVal();
    // drone drives towards the line
    if (drive_to_line) {
      switch (trackVal)
      {
        case 0:   //000 aka detects no black line
          if (black_line_found) {
            motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
            delay(DRIVE_TO_LINE_DELAY);
            motorRun(-TK_FORWARD_SPEED, TK_TURN_SPEED_LV1); // car turn back up left
            delay(RETREAT_DELAY);
          } 
          motorRun(TK_FORWARD_SPEED-5,TK_FORWARD_SPEED-5);
          delay(DRIVE_TO_LINE_DELAY);
          break;
        case 7:   //111 aka finds perpendicular black line
          black_line_found == true;
          motorRun(TK_STOP_SPEED, TK_STOP_SPEED);
          delay(DRIVE_TO_LINE_DELAY);
          motorRun(-TK_FORWARD_SPEED, TK_TURN_SPEED_LV1); // car turn back up left
          delay(RETREAT_DELAY);
          break;
        case 1:   //001
          black_line_found == true;
          motorRun(-TK_FORWARD_SPEED, TK_TURN_SPEED_LV1); // car turn back up left
          delay(RETREAT_DELAY);
          break;
        case 3:   //011
          black_line_found == true;
          motorRun(-TK_FORWARD_SPEED, TK_TURN_SPEED_LV1); // car turn back up left
          delay(RETREAT_DELAY);
          break;
        case 2:   //010
            resetCarAction();
            follow_line = true;
            drive_to_line = false;
            // setBuzzer(true);
            // delay(500);
            alarm(2,2);
            resetCarAction();
            prevTrackVal = trackVal;
            trackVal = getTrackingSensorVal();
            if (trackVal == 0) {
              motorRun(-TK_FORWARD_SPEED, TK_TURN_SPEED_LV1); // car turn back up left
            }
            continue;
            break;
        case 5:   //101
          break;
        case 6:   //110
          black_line_found == true;
          motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); //car turn right fast
          delay(DRIVE_TO_LINE_DELAY);
          break;
        case 4:   //100
          black_line_found == true;
          motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED); //car move forward
          delay(DRIVE_TO_LINE_DELAY);
          break;
        default:
          break;
      }
    }
    if (follow_line) {
      switch (trackVal) {
        case 7:   //111
          motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
          delay(FOLLOW_LINE_DELAY);
          alarm(2,2);
          follow_line = false;
          if (parkingID == 0) {
              setBuzzer(true);
              delay(1000);
              resetCarAction();
              dock_in_base = false;
              is_parked = true;
          } else {
            dock_in_base = true;
            motorRun(TK_FORWARD_SPEED+20,TK_FORWARD_SPEED+20);
            delay(PARKING_DELAY);
          }
          
          break;
        case 1:   //001
          motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); //car turn right fast
          delay(FOLLOW_LINE_DELAY);
          break;
         case 3:   //011
          motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); //car turn right slow
          delay(FOLLOW_LINE_DELAY);
          break;
        case 0:   //000 no line detected
            break;
        case 2:   //010
        case 5:   //101
          motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);  //car move forward
          delay(FOLLOW_LINE_DELAY);
          break;
        case 6:   //110
          motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3); //car turn left slow
          delay(FOLLOW_LINE_DELAY);
          break;
        case 4:   //100
          motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); //car turn left fast
          delay(FOLLOW_LINE_DELAY);
          break;
        default:
          break;
      }
    }
    if (dock_in_base) {
      switch (trackVal) {
        case 7:   //111
          if (prevTrackVal < 7 && prevTrackVal > 0) {
            if (parkingSpot == parkingID) { // parking spot found
              motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
              delay(DOCK_IN_BASE_DELAY);
              alarm(3,1); // alert that parking spot was found
              resetCarAction();
              is_parked = true;
              dock_in_base = false; // car will do nothing
              break; // leave dock_in_base to main while loop
            } else { // parking spot not yet found
              parkingSpot++;
            } 
          }
          motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED); // car move forward
          delay(DOCK_IN_BASE_DELAY);
          break;
        case 1:   //001
          motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); //car turn right fast
          delay(DOCK_IN_BASE_DELAY);
          break;
         case 3:   //011
          motorRun(TK_TURN_SPEED_LV3, TK_TURN_SPEED_LV2); //car turn right slow
          delay(DOCK_IN_BASE_DELAY);
          break;
        case 0:   //000 no line detected
          break;
        case 2:   //010
        case 5:   //101
          motorRun(TK_FORWARD_SPEED, TK_FORWARD_SPEED);  //car move forward
          delay(DOCK_IN_BASE_DELAY);
          break;
        case 6:   //110
          motorRun(TK_TURN_SPEED_LV2, TK_TURN_SPEED_LV3); //car turn left slow
          delay(DOCK_IN_BASE_DELAY);
          break;
        case 4:   //100
          motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); //car turn left fast
          delay(DOCK_IN_BASE_DELAY);
          break;
        default:
          break;
      }
    }
  }

  return;
}

void upLoadVoltageToApp() {
  int voltage = 0;
  if (getBatteryVoltage()) {
    voltage = batteryVoltage * 1000;
  }
  String sendString = String(ACTION_GET_VOLTAGE) + String(INTERVAL_CHAR) + String((voltage)) + String(INTERVAL_CHAR);
  Serial.println(sendString);
}

// bluetooth commands
void serialEvent() {
  if (ignore_BT) return;
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputStringBLE += inChar;
    if (inChar == '\n') {
      stringComplete = true;
      if (stringComplete) {
        String inputCommandArray[COMMANDS_COUNT_MAX];
        parameters[COMMANDS_COUNT_MAX] = 0;
        parameterCount = 0;
        String inputStringTemp = inputStringBLE;
        for (u8 i = 0; i < COMMANDS_COUNT_MAX; i++) {
          int index = inputStringTemp.indexOf(INTERVAL_CHAR);
          if (index < 0) {
            break;
          }
          parameterCount = i;  //
          inputCommandArray[i] = inputStringTemp.substring(0, index);
          inputStringTemp = inputStringTemp.substring(index + 1);
          parameters[i] = inputCommandArray[i].toInt();
        }
        Serial.println(inputStringBLE);
        stringComplete = false;
        inputStringBLE = "";
        commandBT = inputCommandArray[0].charAt(0);
      }
    }
  }
}

void tk_CalculateVoltageCompensation() {
  getBatteryVoltage();
  float voltageOffset = 7 - batteryVoltage;
  tk_Vcomp = 30.0 * voltageOffset;
}
//when black line on one side is detected, the value of the side will be 0, or the value is 1  
u8 getTrackingSensorVal() {
  u8 trackingSensorVal = 0;
  trackingSensorVal = (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 | (digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | (digitalRead(PIN_TRACKING_RIGHT) == 1 ? 1 : 0) << 0;
  return trackingSensorVal;
}

void process_command(int command){
  new_command = 0;

  if (ignore_I2C) return;

  if (turn_dir == 0) {
    switch (command) {
      case 1:
        // Serial.println("Master Control Enabled");
        master_control = true;
        turn_dir = 0; 
        return;
        break;
      case 37:
        //Serial.println("Master Control Disabled");
        master_control = false;
        return;
        break;
      case 38: // no parking id given
        //Serial.println("Drive until line is found");
        drive_to_line = true;
        return;
        break;
      case 39: // parking id 1
        drive_to_line = true;
        parkingID = 1;
        return;
        break;
      case 40: // parking id 2
        drive_to_line = true;
        parkingID = 2;
        return;
        break;
    }
    if(master_control) {
      if(command == 2){
        turn_dir = 2; 
      }
      else if(command == 3){
        turn_dir = 3; 
      }
      return;
    }
  }

  if(master_control && turn_dir){
    if(turn_dir == 2){
      motorRun(TK_TURN_SPEED_LV4, TK_TURN_SPEED_LV1); //car turn right
      delay(3*command); 
      motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
    }
    else if(turn_dir == 3){
      motorRun(TK_TURN_SPEED_LV1, TK_TURN_SPEED_LV4); //car turn left
      delay(3*command); 
      motorRun(TK_STOP_SPEED, TK_STOP_SPEED); //car stop
    }
    turn_dir = 0;
    return; 
  }
  return;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  if (ignore_I2C) return;

  while(Wire.available()) // loop through all but the last
  {
    x = Wire.read();    // receive byte as an integer
    Serial.println(x);         // print the integer
    new_command = 1; 
  }
}
// function from drone kit that requests status of drone
void requestStatus() {
        if (ignore_I2C) return;
        Wire.write(in_recovery | (is_parked << 1));
}