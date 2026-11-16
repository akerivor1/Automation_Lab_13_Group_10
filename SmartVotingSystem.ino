#include <Wire.h>
#include <Servo.h>
#include <IRremote.hpp>
#include <VL53L1X.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// ==========================================
// SMART VOTING SYSTEM
// Arduino UNO - Physical Prototype
// ==========================================

// ---------- PIN DEFINITIONS ----------
const byte SERVO_PIN = 6;
const byte BUTTON_PIN = 7;
const byte LED_PIN = 8;
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;
const byte IR_PIN = 11;

// ---------- DISTANCE THRESHOLDS ----------
const int LASER_MIN_DISTANCE = 80;   // mm = 8 cm
const int LASER_MAX_DISTANCE = 500;  // mm = 50 cm

const float EXIT_DISTANCE = 30.0;   // cm

// ---------- TIME SETTINGS ----------
const unsigned long GATE_WAIT = 7000;
const unsigned long MOTOR_RUN_TIME = 500;
const unsigned long BLINK_INTERVAL = 500;
const unsigned long DEBOUNCE_TIME = 30;

// ---------- CONTINUOUS SERVO ----------
const int MOTOR_STOP = 1500;
const int MOTOR_OPEN = 1700;
const int MOTOR_CLOSE = 1300;

// ---------- IR REMOTE ----------
const byte HELP_KEY = 0x45;    // Button 1
const byte CANCEL_KEY = 0x40;  // Button 5

// ---------- COMPONENT OBJECTS ----------
Servo motor;
VL53L1X laser;
hd44780_I2Cexp lcd;

// ---------- SYSTEM STATES ----------
enum State {
  WAITING,
  WAITING_FOR_BUTTON,
  ENTRY_OPEN,
  VOTER_INSIDE,
  EXIT_OPEN
};

// ---------- MOTOR STATES ----------
enum MotorAction {
  MOTOR_IDLE,
  MOTOR_OPENING,
  MOTOR_CLOSING
};

// ---------- GLOBAL VARIABLES ----------
State state = WAITING;
MotorAction motorAction = MOTOR_IDLE;

bool systemReady = false;
bool helpRequested = false;
bool ledOn = false;
bool lastButton = HIGH;

byte laserCount = 0;

unsigned long gateOpenedAt = 0;
unsigned long motorStartedAt = 0;
unsigned long blinkAt = 0;
unsigned long lastButtonPressAt = 0;

// ---------- FUNCTION DECLARATIONS ----------
void showLCD(const char* line1, const char* line2);
void refreshLCD();

void startMotor(int command, MotorAction action);
void updateMotor();
void openGate();
void closeGate();
void updateGate();

void checkLaser();
float getDistance();
void checkExit();
void checkButton();
void checkRemote();
void updateLED();


// ==========================================
// SETUP - INITIALIZATION
// ==========================================

// Initializes the Arduino components,
// sensors, LCD, motor and IR receiver.
// This function runs only once.
void setup() {

  Serial.begin(9600);

  // Configure digital pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TRIG_PIN, LOW);

  // Initialize continuous servo
  motor.attach(SERVO_PIN);
  motor.writeMicroseconds(MOTOR_STOP);

  // Initialize I2C communication
  Wire.begin();

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();

  showLCD("SMART VOTING", "STARTING...");

  // Initialize laser sensor
  laser.setTimeout(200);

  if (!laser.init()) {

    Serial.println("LASER NOT FOUND");

    showLCD("LASER ERROR", "CHECK SENSOR");

    motor.writeMicroseconds(MOTOR_STOP);

    systemReady = false;
    return;
  }

  laser.setDistanceMode(VL53L1X::Long);
  laser.setMeasurementTimingBudget(50000);
  laser.startContinuous(60);

  // Initialize IR receiver
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);

  systemReady = true;

  refreshLCD();

  Serial.println("SMART VOTING READY");
  Serial.println("MOTOR STOPPED");
  Serial.println("IR RECEIVER READY ON PIN 11");
}


// ==========================================
// LOOP - MAIN CONTROL LOOP
// ==========================================

// The main control loop repeatedly checks
// sensors, user commands and system timing.
// No additional application loops are used.
void loop() {

  // Stop control if initialization failed
  if (!systemReady) {
    return;
  }

  // Update motor movement
  updateMotor();

  // Read sensors and commands
  checkLaser();
  checkRemote();
  checkButton();
  checkExit();

  // Manage gate closing time
  updateGate();

  // Update LED indication
  updateLED();
}


// ==========================================
// ADDITIONAL FUNCTIONS
// ==========================================


// Displays two lines of text on the LCD.
// Clears previous text before updating.
void showLCD(const char* line1, const char* line2) {

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  lcd.print(line2);
}


// Updates the LCD according to the
// current state of the voting system.
// Assistance requests have priority.
void refreshLCD() {

  if (helpRequested) {

    showLCD("HELP REQUESTED", "PLEASE WAIT");
  }

  else if (state == WAITING) {

    showLCD("SMART VOTING", "SYSTEM READY");
  }

  else if (state == WAITING_FOR_BUTTON) {

    showLCD("VOTER WAITING", "PLEASE WAIT");
  }

  else if (state == ENTRY_OPEN) {

    showLCD("GATE OPEN", "PLEASE ENTER");
  }

  else if (state == VOTER_INSIDE) {

    showLCD("VOTER INSIDE", "WAITING EXIT");
  }

  else if (state == EXIT_OPEN) {

    showLCD("GATE OPEN", "PLEASE EXIT");
  }
}


// Starts the continuous servo motor.
// The motor direction and start time
// are stored without using delay().
void startMotor(int command, MotorAction action) {

  motorAction = action;
  motorStartedAt = millis();

  motor.writeMicroseconds(command);
}


// Monitors the motor movement duration.
// Stops the motor after 500 milliseconds
// and updates the system state.
void updateMotor() {

  if (motorAction == MOTOR_IDLE) {
    return;
  }

  if (millis() - motorStartedAt < MOTOR_RUN_TIME) {
    return;
  }

  MotorAction completedAction = motorAction;

  motor.writeMicroseconds(MOTOR_STOP);
  motorAction = MOTOR_IDLE;

  // Opening movement completed
  if (completedAction == MOTOR_OPENING) {

    gateOpenedAt = millis();

    Serial.println("MOTOR STOPPED - GATE OPEN");

    refreshLCD();
  }

  // Closing movement completed
  else if (completedAction == MOTOR_CLOSING) {

    Serial.println("MOTOR STOPPED - GATE CLOSED");

    // Entry sequence completed
    if (state == ENTRY_OPEN) {

      state = VOTER_INSIDE;

      Serial.println("VOTER INSIDE");
    }

    // Exit sequence completed
    else if (state == EXIT_OPEN) {

      state = WAITING;
      laserCount = 0;

      Serial.println("VOTER LEFT");
      Serial.println("SYSTEM READY");
    }

    refreshLCD();
  }
}


// Starts the gate-opening movement.
// The servo rotates for 500 milliseconds
// and then stops automatically.
void openGate() {

  if (motorAction != MOTOR_IDLE) {
    return;
  }

  if (!helpRequested) {

    showLCD("GATE OPENING", "PLEASE WAIT");
  }

  Serial.println("GATE OPENING");

  startMotor(MOTOR_OPEN, MOTOR_OPENING);
}


// Starts the gate-closing movement.
// The servo rotates in the opposite
// direction and stops automatically.
void closeGate() {

  if (motorAction != MOTOR_IDLE) {
    return;
  }

  Serial.println("GATE CLOSING");

  startMotor(MOTOR_CLOSE, MOTOR_CLOSING);
}


// Checks whether the gate has remained
// open for 7 seconds after opening.
// Starts closing for both entry and exit.
void updateGate() {

  if (motorAction != MOTOR_IDLE) {
    return;
  }

  if (state == ENTRY_OPEN || state == EXIT_OPEN) {

    if (millis() - gateOpenedAt >= GATE_WAIT) {

      closeGate();
    }
  }
}


// Detects a voter near the laser sensor.
// Valid detection range: 8 to 50 cm.
// Three valid readings are required.
void checkLaser() {

  if (state != WAITING) {
    return;
  }

  if (!laser.dataReady()) {
    return;
  }

  int distance = laser.read(false);

  if (laser.timeoutOccurred()) {
    return;
  }

  if (laser.ranging_data.range_status !=
      VL53L1X::RangeValid) {

    laserCount = 0;
    return;
  }

  if (distance >= LASER_MIN_DISTANCE &&
      distance <= LASER_MAX_DISTANCE) {

    laserCount++;

    if (laserCount >= 3) {

      laserCount = 0;
      state = WAITING_FOR_BUTTON;

      Serial.println("LASER: VOTER DETECTED");
      Serial.println("LED ON - VOTER WAITING");

      refreshLCD();
    }
  }

  else {

    laserCount = 0;
  }
}


// Measures distance using the HC-SR04.
// Returns distance in centimeters.
// Returns -1 when no echo is received.
float getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration =
    pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (duration == 0) {
    return -1;
  }

  float distance = duration * 0.0343 / 2.0;

  return distance;
}


// Detects a voter approaching the exit.
// Opens the gate when a valid distance
// of 30 cm or less is measured.
// No 45 cm condition is required.
void checkExit() {

  if (state != VOTER_INSIDE) {
    return;
  }

  float distance = getDistance();

  if (distance > 0 &&
      distance <= EXIT_DISTANCE) {

    Serial.println("ULTRASONIC: EXIT DETECTED");

    state = EXIT_OPEN;

    openGate();
  }
}


// Reads the secretary's physical button.
// A press approves entry only when
// a voter is waiting outside.
// Uses millis() for simple debouncing.
void checkButton() {

  bool currentButton = digitalRead(BUTTON_PIN);

  if (lastButton == HIGH &&
      currentButton == LOW) {

    if (millis() - lastButtonPressAt >= DEBOUNCE_TIME) {

      lastButtonPressAt = millis();

      if (state == WAITING_FOR_BUTTON) {

        Serial.println("SECRETARY APPROVED ENTRY");

        state = ENTRY_OPEN;

        openGate();
      }
    }
  }

  lastButton = currentButton;
}


// Reads the physical IR remote.
// Button 1 requests accessibility help.
// Button 5 cancels the help request.
// Repeated IR commands are ignored.
void checkRemote() {

  if (!IrReceiver.decode()) {
    return;
  }

  byte command = IrReceiver.decodedIRData.command;

  bool repeat =
    IrReceiver.decodedIRData.flags &
    IRDATA_FLAGS_IS_REPEAT;

  if (!repeat) {

    // Button 1 - Request help
    if (command == HELP_KEY && !helpRequested) {

      helpRequested = true;

      ledOn = false;
      blinkAt = millis();

      Serial.println("HELP REQUESTED");

      refreshLCD();
    }

    // Button 5 - Cancel help
    else if (command == CANCEL_KEY &&
             helpRequested) {

      helpRequested = false;
      ledOn = false;

      Serial.println("HELP CANCELLED");

      refreshLCD();
    }
  }

  IrReceiver.resume();
}


// Controls the LED according to the
// system state.
// Steady ON: voter waiting.
// Blinking: assistance requested.
// OFF: other normal states.
void updateLED() {

  if (helpRequested) {

    if (millis() - blinkAt >= BLINK_INTERVAL) {

      blinkAt = millis();
      ledOn = !ledOn;
    }

    digitalWrite(LED_PIN, ledOn);
  }

  else if (state == WAITING_FOR_BUTTON) {

    digitalWrite(LED_PIN, HIGH);
  }

  else {

    digitalWrite(LED_PIN, LOW);
  }
}
