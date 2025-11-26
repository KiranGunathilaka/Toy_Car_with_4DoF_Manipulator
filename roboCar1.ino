/* 🤖 ESP32 ROBOT CONTROLLER - Easy to Understand Version! 🤖
 * 
 * WHAT THIS CODE DOES:
 * This program lets you control a robot car with a robotic arm using your phone!
 * - You can drive forward, backward, and turn left/right
 * - You can steer the front wheels like a real car
 * - You can move a robotic arm with 4 joints (base, shoulder, elbow, gripper)
 * 
 * HOW TO USE YOUR PHONE CONTROLLER:
 * 
 *  DRIVING (Start button + Left arrows):
 *    Start + Up Arrow    → Go Forward
 *    Start + Down Arrow  → Go Backward  
 *    Start + Left Arrow  → Spin Left (like a tank!)
 *    Start + Right Arrow → Spin Right (like a tank!)
 * 
 *  STEERING (Select button + Left arrows):
 *    Select + Up Arrow    → Point wheels straight
 *    Select + Left Arrow  → Point wheels left
 *    Select + Right Arrow → Point wheels right
 * 
 *  ROBOT ARM - Make it bigger (Select + Right buttons):
 *    Select + Triangle → Move base (bottom joint)
 *    Select + Circle   → Move shoulder (middle joint)
 *    Select + Cross    → Move elbow (upper joint)
 *    Select + Square   → Open/close gripper (hand)
 * 
 *  ROBOT ARM - Make it smaller (Start + Right buttons):
 *    Start + Triangle → Move base back
 *    Start + Circle   → Move shoulder back
 *    Start + Cross    → Move elbow back
 *    Start + Square   → Close/open gripper
 */

// These lines import special code libraries that help us control the robot
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>  // This library talks to your phone via Bluetooth
#include <ESP32Servo.h>    // This library controls the robot arm servos (motors that turn to specific angles)

/************  PIN DEFINITIONS - Where wires connect to the ESP32 board ************/

//  MOTOR PINS (these control the wheels)
const int ENA = 12; // Right motor speed control (how fast it spins)
const int ENB = 33; // Left motor speed control (how fast it spins)
const int IN1 = 14; // Right motor direction pin 1 (forward or backward)
const int IN2 = 27; // Right motor direction pin 2 (forward or backward)
const int IN3 = 26; // Left motor direction pin 1 (forward or backward)
const int IN4 = 25; // Left motor direction pin 2 (forward or backward)

//  SERVO PINS (these control the robot arm)
const int PIN_STEER = 21;    // Controls steering (which way the front wheels point)
const int PIN_BASE = 15;     // Controls the base (bottom rotating part of arm)
const int PIN_SHOULDER = 18; // Controls the shoulder (middle joint of arm)
const int PIN_ELBOW = 19;    // Controls the elbow (upper joint of arm)
const int PIN_GRIP = 23;     // Controls the gripper (the hand that grabs things)

/************  BUTTON CODE DEFINITIONS - Secret numbers from your phone controller ************/
// When you press buttons on your phone, it sends special numbers to the robot.
// We need to know what each number means!

// Basic button codes
const int CODE_ALL_RELEASED = 274;   // No buttons pressed - robot should stop
const int CODE_START_PRESSED = 280;  // Only Start button is pressed
const int CODE_SELECT_PRESSED = 286; // Only Select button is pressed

//  DRIVING CODES (Start button + arrow buttons)
const int CODE_DRIVE_FORWARD = 287; // Start + Up arrow
const int CODE_DRIVE_BACK = 294;    // Start + Down arrow
const int CODE_DRIVE_LEFT = 336;    // Start + Left arrow (spins left)
const int CODE_DRIVE_RIGHT = 308;   // Start + Right arrow (spins right)

//  STEERING CODES (Select button + arrow buttons)
const int CODE_STEER_CENTER = 293;  // Select + Up arrow (straighten wheels)
const int CODE_STEER_CENTER2 = 300; // Select + Down arrow (also straightens wheels)
const int CODE_STEER_LEFT = 314;    // Select + Left arrow (turn wheels left)
const int CODE_STEER_RIGHT = 342;   // Select + Right arrow (turn wheels right)

//  ARM CODES - INCREASE angles (Select button + shape buttons on right)
const int CODE_ARM_INC_SERVO1 = 310; // Select + Triangle (move base one way)
const int CODE_ARM_INC_SERVO2 = 334; // Select + Circle (move shoulder one way)
const int CODE_ARM_INC_SERVO3 = 382; // Select + Cross (move elbow one way)
const int CODE_ARM_INC_SERVO4 = 478; // Select + Square (open gripper)

//  ARM CODES - DECREASE angles (Start button + shape buttons on right)
const int CODE_ARM_DEC_SERVO1 = 304; // Start + Triangle (move base other way)
const int CODE_ARM_DEC_SERVO2 = 328; // Start + Circle (move shoulder other way)
const int CODE_ARM_DEC_SERVO3 = 376; // Start + Cross (move elbow other way)
const int CODE_ARM_DEC_SERVO4 = 472; // Start + Square (close gripper)

/************ MOTOR & SERVO SETTINGS ************/

// Motor speed settings (0 to 255, where 255 is fastest)
const int PWM_MAX = 255;       // Maximum speed possible
const int DRIVE_SPEED = 255;   // Speed when driving forward/backward
const int TURN_SPEED = 220;    // Speed when spinning left/right

// Steering servo angles (in degrees, like a protractor)
const int STEER_CENTER = 90;       // Wheels pointing straight (middle position)
const int STEER_LEFT_ANGLE = 50;   // Wheels turned left
const int STEER_RIGHT_ANGLE = 130; // Wheels turned right

// Robot arm servo limits (minimum and maximum angles for safety)
// These stop the servos from breaking by moving too far!
const int BASE_MIN = 10;      // Base can't go below 10 degrees
const int BASE_MAX = 170;     // Base can't go above 170 degrees
const int SHOULDER_MIN = 10;  // Shoulder minimum angle
const int SHOULDER_MAX = 170; // Shoulder maximum angle
const int ELBOW_MIN = 30;     // Elbow minimum angle
const int ELBOW_MAX = 150;    // Elbow maximum angle
const int GRIP_MIN = 10;      // Gripper minimum angle (closed)
const int GRIP_MAX = 170;     // Gripper maximum angle (open)

// How much the arm moves with each button press
const int ARM_STEP = 10; // Move 10 degrees each time you press a button
const int ELBOW_STEP = 80;

// PWM channels for motors (these are like TV channels but for motor signals)
const int CH_ENA = 6; // Channel for right motor
const int CH_ENB = 7; // Channel for left motor

/************  SERVO OBJECTS - These represent each moving part ************/
// Think of these as the "brains" that control each servo motor
Servo steerServo;    // Controls the steering
Servo baseServo;     // Controls the arm base
Servo shoulderServo; // Controls the arm shoulder
Servo elbowServo;    // Controls the arm elbow
Servo gripperServo;  // Controls the gripper

/************  STATE VARIABLES - The robot's memory ************/
// These variables remember the current position of everything

// Current servo positions (in degrees)
int steerPos = STEER_CENTER; // Where the steering is pointing (starts at 90 = center)
int basePos = 110;            // Where the base is pointing (starts at middle)
int shoulderPos = 100;        // Where the shoulder is (starts at middle)
int elbowPos = 100;           // Where the elbow is (starts at middle)
int gripperPos = 120;         // How open the gripper is (starts at middle)

// Variables to track what the robot is doing right now
int lastCode = 0;              // The last button code we received
int currentDriveCommand = 0;   // What driving command is active
bool isDriving = false;        // Is the robot moving right now? (true/false)

// Variables to prevent the arm from moving too fast
int lastArmCommand = 0;                      // Last arm button that was pressed
unsigned long lastArmMoveTime = 0;           // When did we last move the arm?
const unsigned long ARM_REPEAT_DELAY = 200;  // Wait 200 milliseconds between arm moves

/************  HELPER FUNCTIONS - Useful tools the robot uses ************/

/* This function makes sure a number stays within a range
 * Example: If you try to set servo to 200 degrees but max is 170,
 * it will return 170 instead to keep the servo safe!
 */
int clampValue(int value, int minVal, int maxVal)
{
  if (value < minVal)    // If value is too small
    return minVal;       // Return the minimum instead
  if (value > maxVal)    // If value is too big
    return maxVal;       // Return the maximum instead
  return value;          // If value is just right, return it as-is
}

/* This function tells one motor what to do
 * It controls both SPEED (how fast) and DIRECTION (forward or backward)
 */
void setMotor(int enaPin, int inA, int inB, int speed, int direction)
{
  // First, make sure speed is between 0 and 255
  speed = clampValue(speed, 0, PWM_MAX);

  // Set the direction using the IN pins
  if (direction > 0)  // Positive number = forward
  {
    digitalWrite(inA, HIGH); // Turn on pin A
    digitalWrite(inB, LOW);  // Turn off pin B
  }
  else if (direction < 0)  // Negative number = backward
  {
    digitalWrite(inA, LOW);  // Turn off pin A
    digitalWrite(inB, HIGH); // Turn on pin B
  }
  else  // Zero = stop
  {
    digitalWrite(inA, LOW);  // Turn off both pins
    digitalWrite(inB, LOW);  // Motor will stop
  }

  // Now set the speed using PWM (Pulse Width Modulation)
  if (enaPin == ENA)  // If this is the right motor
  {
    ledcWrite(CH_ENA, speed);  // Set right motor speed
  }
  else if (enaPin == ENB)  // If this is the left motor
  {
    ledcWrite(CH_ENB, speed);  // Set left motor speed
  }
}

/* STOP both motors completely */
void stopMotors()
{
  setMotor(ENA, IN1, IN2, 0, 0);  // Stop right motor
  setMotor(ENB, IN3, IN4, 0, 0);  // Stop left motor
  isDriving = false;               // Remember we're not driving anymore
  currentDriveCommand = 0;         // Clear the current drive command
  Serial.println("Motors STOPPED");  // Print message so we can see what's happening
}

/* Make the robot drive FORWARD */
void driveForward()
{
  // Note: The motors are wired backward, so -1 actually makes them go forward!
  setMotor(ENA, IN1, IN2, DRIVE_SPEED, -1); // Right motor forward
  setMotor(ENB, IN3, IN4, DRIVE_SPEED, -1); // Left motor forward
  isDriving = true;  // Remember we're driving now
  Serial.println("Driving FORWARD");  // Print message
}

/* Make the robot drive BACKWARD */
void driveBackward()
{
  // Reverse direction from forward
  setMotor(ENA, IN1, IN2, DRIVE_SPEED, 1);  // Right motor backward
  setMotor(ENB, IN3, IN4, DRIVE_SPEED, 1);  // Left motor backward
  isDriving = true;  // Remember we're driving now
  Serial.println("← Driving BACKWARD");  // Print message
}

/* Spin the robot LEFT (like a tank turning)
 * Left motor goes forward, right motor goes backward
 * This makes the robot spin in place!
 */
void turnLeft()
{
  setMotor(ENA, IN1, IN2, TURN_SPEED, -1); // Right motor backward
  setMotor(ENB, IN3, IN4, TURN_SPEED, 1);  // Left motor forward
  isDriving = true;  // Remember we're driving now
  Serial.println("↺ Turning LEFT (Differential)");  // Print message
}

/* Spin the robot RIGHT (like a tank turning)
 * Right motor goes forward, left motor goes backward
 */
void turnRight()
{
  setMotor(ENA, IN1, IN2, TURN_SPEED, 1);  // Right motor forward
  setMotor(ENB, IN3, IN4, TURN_SPEED, -1); // Left motor backward
  isDriving = true;  // Remember we're driving now
  Serial.println("↻ Turning RIGHT (Differential)");  // Print message
}

/* Point the steering wheels STRAIGHT */
void steerCenter()
{
  steerPos = STEER_CENTER;      // Set position to 90 degrees (center)
  steerServo.write(steerPos);   // Tell the servo to move there
  Serial.println("⊙ Steering CENTER");  // Print message
}

/* Point the steering wheels LEFT */
void steerLeft()
{
  steerPos = STEER_LEFT_ANGLE;  // Set position to 60 degrees
  steerServo.write(steerPos);   // Tell the servo to move there
  Serial.println("⟲ Steering LEFT");  // Print message
}

/* Point the steering wheels RIGHT */
void steerRight()
{
  steerPos = STEER_RIGHT_ANGLE; // Set position to 120 degrees
  steerServo.write(steerPos);   // Tell the servo to move there
  Serial.println("⟳ Steering RIGHT");  // Print message
}

/* Update ALL arm servos to their current positions
 * This sends the angle values to all 4 arm servos at once
 */
void updateArmServos()
{
  baseServo.write(basePos);         // Move base to its position
  shoulderServo.write(shoulderPos); // Move shoulder to its position
  elbowServo.write(elbowPos);       // Move elbow to its position
  gripperServo.write(gripperPos);   // Move gripper to its position
}

/* Check if a code is a DRIVING command */
bool isDriveCommand(int code)
{
  return (code == CODE_DRIVE_FORWARD || code == CODE_DRIVE_BACK ||
          code == CODE_DRIVE_LEFT || code == CODE_DRIVE_RIGHT);
}

/* Check if a code is a STEERING command */
bool isSteerCommand(int code)
{
  return (code == CODE_STEER_CENTER || code == CODE_STEER_CENTER2 ||
          code == CODE_STEER_LEFT || code == CODE_STEER_RIGHT);
}

/* Check if a code is an ARM command */
bool isArmCommand(int code)
{
  return (code == CODE_ARM_INC_SERVO1 || code == CODE_ARM_INC_SERVO2 ||
          code == CODE_ARM_INC_SERVO3 || code == CODE_ARM_INC_SERVO4 ||
          code == CODE_ARM_DEC_SERVO1 || code == CODE_ARM_DEC_SERVO2 ||
          code == CODE_ARM_DEC_SERVO3 || code == CODE_ARM_DEC_SERVO4);
}

/*  THE BRAIN - This function decides what to do based on button codes
 * It's like a translator that converts button presses into robot actions!
 */
void processButtonCode(int code)
{
  // ===== HANDLE: ALL BUTTONS RELEASED =====
  // If you let go of all buttons, stop everything!
  if (code == CODE_ALL_RELEASED)
  {
    if (isDriving)  // If motors were running
    {
      stopMotors();  // Stop them
    }
    currentDriveCommand = 0;  // Clear drive command
    lastArmCommand = 0;       // Clear arm command
    lastCode = code;          // Remember this code
    return;  // Exit this function
  }

  // ===== HANDLE: ONLY START BUTTON PRESSED =====
  // If only Start is pressed (no combination), stop driving
  if (code == CODE_START_PRESSED)
  {
    if (isDriving)  // If motors were running
    {
      stopMotors();  // Stop them
    }
    currentDriveCommand = 0;  // Clear drive command
    lastCode = code;          // Remember this code
    return;  // Exit this function
  }

  // ===== HANDLE: ONLY SELECT BUTTON PRESSED =====
  // If only Select is pressed (no combination), do nothing special
  if (code == CODE_SELECT_PRESSED)
  {
    lastCode = code;  // Just remember this code
    return;  // Exit this function
  }

  // ===== DRIVING CONTROL (Start + Arrow buttons) =====
  if (isDriveCommand(code))  // Is this a driving command?
  {
    if (code != currentDriveCommand)  // Is it different from what we're doing now?
    {
      currentDriveCommand = code;  // Remember new command

      // Do the right action based on which button was pressed
      if (code == CODE_DRIVE_FORWARD)
      {
        driveForward();  // Go forward
      }
      else if (code == CODE_DRIVE_BACK)
      {
        driveBackward();  // Go backward
      }
      else if (code == CODE_DRIVE_LEFT)
      {
        turnLeft();  // Spin left
      }
      else if (code == CODE_DRIVE_RIGHT)
      {
        turnRight();  // Spin right
      }
    }
    // If code equals currentDriveCommand, keep doing what we're doing (don't repeat the command)
  }

  // ===== STEERING CONTROL (Select + Arrow buttons) =====
  else if (isSteerCommand(code))  // Is this a steering command?
  {
    // Do the right action based on which button was pressed
    if (code == CODE_STEER_CENTER || code == CODE_STEER_CENTER2)
    {
      steerCenter();  // Point wheels straight
    }
    else if (code == CODE_STEER_LEFT)
    {
      steerLeft();  // Point wheels left
    }
    else if (code == CODE_STEER_RIGHT)
    {
      steerRight();  // Point wheels right
    }
  }

  // ===== ARM CONTROL (Select/Start + Shape buttons) =====
  else if (isArmCommand(code))  // Is this an arm command?
  {
    unsigned long currentTime = millis();  // Get current time in milliseconds
    
    // Should we move the arm? Only if:
    // 1. It's a NEW command (different button), OR
    // 2. Enough time has passed since last move (prevents too-fast movement)
    bool shouldMove = (code != lastArmCommand) ||
                      (currentTime - lastArmMoveTime >= ARM_REPEAT_DELAY);

    if (shouldMove)  // OK to move the arm?
    {
      lastArmCommand = code;          // Remember this command
      lastArmMoveTime = currentTime;  // Remember when we moved

      // ===== INCREASE COMMANDS (Select + Shape buttons) =====
      // These make the servo angles BIGGER (more open/extended)
      
      if (code == CODE_ARM_INC_SERVO1)  // Select + Triangle
      {
        basePos = clampValue(basePos + ARM_STEP, BASE_MIN, BASE_MAX);  // Add 10 degrees (but stay within limits)
        updateArmServos();  // Move all servos
        Serial.print("Base ↑ → ");  // Print what we did
        Serial.println(basePos);    // Print new position
      }
      else if (code == CODE_ARM_INC_SERVO2)  // Select + Circle
      {
        shoulderPos = clampValue(shoulderPos + ARM_STEP, SHOULDER_MIN, SHOULDER_MAX);
        updateArmServos();
        Serial.print("Shoulder ↑ → ");
        Serial.println(shoulderPos);
      }
      else if (code == CODE_ARM_INC_SERVO3)  // Select + Cross
      {
        elbowPos = clampValue(elbowPos + ELBOW_STEP, ELBOW_MIN, ELBOW_MAX);
        updateArmServos();
        Serial.print("Elbow ↑ → ");
        Serial.println(elbowPos);
      }
      else if (code == CODE_ARM_INC_SERVO4)  // Select + Square
      {
        gripperPos = clampValue(gripperPos + ARM_STEP, GRIP_MIN, GRIP_MAX);
        updateArmServos();
        Serial.print("Gripper ↑ → ");
        Serial.println(gripperPos);
      }

      // ===== DECREASE COMMANDS (Start + Shape buttons) =====
      // These make the servo angles SMALLER (more closed/retracted)
      
      else if (code == CODE_ARM_DEC_SERVO1)  // Start + Triangle
      {
        basePos = clampValue(basePos - ARM_STEP, BASE_MIN, BASE_MAX);  // Subtract 10 degrees (but stay within limits)
        updateArmServos();  // Move all servos
        Serial.print("Base ↓ → ");  // Print what we did
        Serial.println(basePos);    // Print new position
      }
      else if (code == CODE_ARM_DEC_SERVO2)  // Start + Circle
      {
        shoulderPos = clampValue(shoulderPos - ARM_STEP, SHOULDER_MIN, SHOULDER_MAX);
        updateArmServos();
        Serial.print("Shoulder ↓ → ");
        Serial.println(shoulderPos);
      }
      else if (code == CODE_ARM_DEC_SERVO3)  // Start + Cross
      {
        elbowPos = clampValue(elbowPos - ARM_STEP, ELBOW_MIN, ELBOW_MAX);
        updateArmServos();
        Serial.print("Elbow ↓ → ");
        Serial.println(elbowPos);
      }
      else if (code == CODE_ARM_DEC_SERVO4)  // Start + Square
      {
        gripperPos = clampValue(gripperPos - ARM_STEP, GRIP_MIN, GRIP_MAX);
        updateArmServos();
        Serial.print("Gripper ↓ → ");
        Serial.println(gripperPos);
      }
    }
  }

  lastCode = code;  // Remember this code for next time
}

/************ 🚀 SETUP - Runs ONCE when robot starts ************/
void setup()
{
  // Start serial communication (this lets us see messages on the computer)
  Serial.begin(115200);  // 115200 is the speed (like internet speed but for serial)
  delay(500);  // Wait half a second for everything to stabilize

  // Print welcome messages
  Serial.println("\n\n========================================");
  Serial.println("ESP32 Robot Controller Starting...");
  Serial.println("ACKERMANN + DIFFERENTIAL STEERING");
  Serial.println("========================================");

  // ===== SETUP SERVOS =====
  Serial.println("Initializing servos...");

  // Attach each servo to its pin (connect servo to ESP32)
  steerServo.attach(PIN_STEER);      // Connect steering servo
  delay(100);  // Small delay between each servo
  baseServo.attach(PIN_BASE);        // Connect base servo
  delay(100);
  shoulderServo.attach(PIN_SHOULDER);// Connect shoulder servo
  delay(100);
  elbowServo.attach(PIN_ELBOW);      // Connect elbow servo
  delay(100);
  gripperServo.attach(PIN_GRIP);     // Connect gripper servo
  delay(100);

  // Move all servos to their starting positions
  steerServo.write(steerPos);        // Move steering to center (90°)
  baseServo.write(basePos);          // Move base to middle (90°)
  shoulderServo.write(shoulderPos);  // Move shoulder to middle (90°)
  elbowServo.write(elbowPos);        // Move elbow to middle (90°)
  gripperServo.write(gripperPos);    // Move gripper to middle (90°)

  // Print success message with pin numbers
  Serial.println("✓ All servos initialized");
  Serial.print("  Steering: Pin ");
  Serial.println(PIN_STEER);
  Serial.print("  Base: Pin ");
  Serial.println(PIN_BASE);
  Serial.print("  Shoulder: Pin ");
  Serial.println(PIN_SHOULDER);
  Serial.print("  Elbow: Pin ");
  Serial.println(PIN_ELBOW);
  Serial.print("  Gripper: Pin ");
  Serial.println(PIN_GRIP);

  // ===== SETUP MOTOR PWM =====
  // PWM is how we control motor speed (like dimming a light)
  ledcSetup(CH_ENA, 20000, 8);  // Setup channel 0: 20kHz frequency, 8-bit resolution
  ledcAttachPin(ENA, CH_ENA);   // Attach channel 0 to right motor speed pin
  ledcSetup(CH_ENB, 20000, 8);  // Setup channel 1: 20kHz frequency, 8-bit resolution
  ledcAttachPin(ENB, CH_ENB);   // Attach channel 1 to left motor speed pin

  // ===== SETUP MOTOR PINS =====
  // Set all motor pins as outputs (they send signals OUT)
  pinMode(ENA, OUTPUT);  // Right motor speed
  pinMode(ENB, OUTPUT);  // Left motor speed
  pinMode(IN1, OUTPUT);  // Right motor direction 1
  pinMode(IN2, OUTPUT);  // Right motor direction 2
  pinMode(IN3, OUTPUT);  // Left motor direction 1
  pinMode(IN4, OUTPUT);  // Left motor direction 2

  stopMotors();  // Make sure motors start in STOP mode
  Serial.println("✓ Motors initialized (Differential Drive)");

  // ===== SETUP BLUETOOTH =====
  Dabble.begin("TechnoBlade Robot 1");  // Start Bluetooth with this name (you'll see this on your phone)
  Serial.println("✓ Dabble Bluetooth initialized");

  // Print final instructions
  Serial.println("\n Robot Ready!");
  Serial.println(" Drive: Start + Left D-pad (Front/Back/Left/Right)");
  Serial.println(" Steer: Select + Left D-pad (Front=Center, Left, Right)");
  Serial.println(" Arm +: Select + Right Shape buttons (△○✕□)");
  Serial.println(" Arm -: Start + Right Shape buttons (△○✕□)");
  Serial.println("\n  If servos don't work, check wiring to pins:");
  Serial.println("   Steer=13, Base=15, Shoulder=18, Elbow=19, Grip=23");
  Serial.println("Waiting for connection...\n");
}

/************  LOOP - Runs FOREVER (over and over) ************/
void loop()
{
  // Check if any button was pressed on the phone
  int code = Dabble.processInput();  // Get the button code from phone

  if (code > 0)  // If we got a valid code (not zero)
  {
    // Print the code so we can see what's happening
    Serial.print("Code: ");
    Serial.print(code);
    Serial.print(" | ");
    
    // Process the code and make the robot do something!
    processButtonCode(code);
  }

  delay(50);  // Wait 50 milliseconds before checking again (prevents too-fast checking)
}

/*  That's it! The robot will keep looping forever, checking for button presses
 * and making the robot move based on what you press!  */