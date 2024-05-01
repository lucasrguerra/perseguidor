/****************************************************************
* Folow Line - Perseguidor
* Garagino - CESAR School
* 
* Line Follower PID with the microcontroller Vespa 
* from RoboCore and the Pololu's QTR-8RC sensor
****************************************************************/

#define DEBUG                     // Debug mode
#define BT_NAME "Motoneta"        // Names: Mutuca | Motoneta | Van Dyne



#ifdef DEBUG
  #include "BluetoothSerial.h"    // Bluetooth Serial library
  BluetoothSerial SerialBT;       // Bluetooth Serial instance

  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif
#endif



#include <RoboCore_Vespa.h>       // Library for the Vespa microcontroller
#include <QTRSensors.h>           // Library for the QTR-8A or the QTR-8RC



// ----------------- Objects ----------------- 
VespaMotors motor;                // Vespa Motor Object
QTRSensors qtr;                   // QTR Sensor
// -------------------------------------------


// ----------------- Constants -----------------
const uint8_t PIN_BUTTON = 35;
const uint8_t PIN_LED = 15;
const uint8_t PIN_MARKER_SENSOR = 36;
const uint8_t SENSOR_COUNT = 8;
const int16_t MAX_POSITION = (SENSOR_COUNT - 1) * 1000;
const uint8_t PINS_SENSOR[SENSOR_COUNT] = { 21, 19, 5, 16, 22, 23, 18, 17 };
const bool LINE_BLACK = false;
// ---------------------------------------------



// ----------------- Variables -----------------
uint16_t sensorValues[SENSOR_COUNT];
uint32_t startMakerChecker = 39500L;
uint32_t initialTime;
uint8_t marginError = 20;
bool firstRun = true;
// ---------------------------------------------



// ----------------- PID Control -----------------
float p = 0;
float i = 0;
float d = 0;
float pid = 0;
float error = 0;
float lastError = 0;

float Kp = 0.3;
float Ki = 0.0001;
float Kd = 3.5;

int maxSpeed = 100;
int leftSpeed;
int rightSpeed;
// -----------------------------------------------



// ----------------- Functions -----------------
void calibrateSensors();
int readSensors();
bool markerChecker();

#ifdef DEBUG
  String receiveBluetoothMessage();
  String getElement(String data, int index);
  void setupBluetooth();
  void printParametersInBluetooth();
  void printCalibrationInBluetooth();
#endif
// ---------------------------------------------



void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins(PINS_SENSOR, SENSOR_COUNT);

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_MARKER_SENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);

  #ifdef DEBUG
    if (firstRun) {
      Serial.begin(115200);
      delay(100);

      SerialBT.begin(BT_NAME);
      firstRun = false;
    }

    setupBluetooth();
  #endif

  calibrateSensors();

  #ifdef DEBUG
    printCalibrationInBluetooth();
  #endif

  delay(2000);
  initialTime = millis();
}



void loop() {
  error = map(readSensors(), 0, MAX_POSITION, -1000, 1000);

  // Calculate PID
  p = error;
  i = i + error;
  d = error - lastError;
  pid = (Kp * p) + (Ki * i) + (Kd * d);
  lastError = error;

  // Control the motors
  leftSpeed = maxSpeed + pid;
  rightSpeed = maxSpeed - pid;
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  if (markerChecker()) {    // Count the markers and stop the robot when reach a certain number
    motor.stop();

    #ifdef DEBUG
      SerialBT.print(">> Timelapse: ");
      SerialBT.print(millis() - initialTime);
      SerialBT.println(" seconds");
    #endif

    setup();
  } else if (error >= -marginError && error <= marginError) {
    motor.turn(maxSpeed, maxSpeed);
  } else {
    motor.turn(rightSpeed, leftSpeed);
  }
}



void calibrateSensors() {
  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {
    qtr.calibrate(QTR_EMITTERS_ON);
  }
  digitalWrite(PIN_LED, LOW);
}



/**
  * Read the sensors and return the line position
  * 
  * @return The line position between 0 and `MAX_POSITION`.
*/
int readSensors() {
  if (LINE_BLACK) { return qtr.readLineBlack(sensorValues); }
  else { return qtr.readLineWhite(sensorValues); }
}



/**
  * Verifies if there is a end line after a set time
  * 
  * @return `true` if the end line was detected.
*/
bool markerChecker() {
  if (startMakerChecker < millis() - initialTime) {
    if (analogRead(PIN_MARKER_SENSOR) < 2000) {
      return true;
    }
  }

  return false;
}



#ifdef DEBUG  // Functions for debug mode

/**
  * Returns all stream of data sent over bluetooth until the
  * button is pressed.
  *
  * @return `String` with the message sent by the bluetooth device
*/
String receiveBluetoothMessage() {
  String message;
  char incomingChar;

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {
    if (SerialBT.available()) {
      incomingChar = SerialBT.read();

      if (incomingChar == '\n') break;

      message += String(incomingChar);
    }
  }
  digitalWrite(PIN_LED, LOW);

  message.trim();
  return message;
}



/**
  * Returns a sub-string in the `String` data, in the index
  * position.
  *
  * @param `data` String with the message
  * @param `index` Position of the element to be returned
  * @return `String` sub-string in the indicated position. If there is no value at this position, it returns empty string.
*/
String getElement(String data, int index) {
  char separator = ' ';
  int found = 0;
  int startIndex = 0, endIndex = -1;
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      startIndex = endIndex + 1;
      endIndex = (i == maxIndex) ? i + 1 : i;
    }
  }

  if (found <= index) {
    return "";
  }

  return data.substring(startIndex, endIndex);
}



/**
  * Returns the prefix of the message sent by bluetooth
  *
  * @param `data` String with the message
  * @return `String` with the prefix of the message
*/
String getPrefix(String data) {
  return getElement(data, 0);
}



/**
  * Returns the number in the index position of the message sent by bluetooth
  *
  * @param `data` String with the message
  * @param `index` Position of the number to be returned
  * @return `double` number in the indicated position
*/
double getNumber(String data, int index) {
  return atof(getElement(data, index).c_str());
}



void setupBluetooth() {
  SerialBT.println("Start BT communication");

  String btMessage;
  String prefix;

  while (prefix != "end" && digitalRead(PIN_BUTTON) == HIGH) {
    btMessage = receiveBluetoothMessage();
    prefix = getPrefix(btMessage);

    if (prefix == "pid") {
      Kp = getNumber(btMessage, 1);
      Ki = getNumber(btMessage, 2);
      Kd = getNumber(btMessage, 3);
    }
    else if (prefix == "spe") { maxSpeed = getNumber(btMessage, 1); }
    else if (prefix == "tim") { startMakerChecker = getNumber(btMessage, 1); }
    else if (prefix == "err") { marginError = getNumber(btMessage, 1); }
    else if (prefix == "pri") { printParametersInBluetooth(); }
    else if (prefix == "end") { break; }
    else { SerialBT.println("This command doesn't exists!"); }
  }

  printParametersInBluetooth();
  delay(500);
  SerialBT.println("Start Calibration...");
}



void printParametersInBluetooth() {
  SerialBT.println(f("Configured parameters:"));
  SerialBT.print(">> Kp: ");
  SerialBT.print(Kp, 4);
  SerialBT.print(" | Ki: ");
  SerialBT.print(Ki, 4);
  SerialBT.print(" | Kd: ");
  SerialBT.println(Kd, 4);

  SerialBT.print(">> Max Speed: ");
  SerialBT.print(maxSpeed);

  SerialBT.print(">> Time delay: ");
  SerialBT.print(startMakerChecker);

  SerialBT.print(">> Error margin: ");
  SerialBT.println(marginError);
}



void printCalibrationInBluetooth() {
  SerialBT.println("Wating command ...");
  while( prefix != "go"){
    btMessage = receiveBtMessage();
    prefix = getPrefix(btMessage);
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibratedMinimumOn[i]);
    SerialBT.print(" ");
  }
  SerialBT.println();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibratedMaximumOn[i]);
    SerialBT.print(" ");
  }
  SerialBT.println();
}

#endif        // End of functions for debug mode