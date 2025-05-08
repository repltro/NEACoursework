#include <Servo.h>
#include <AccelStepper.h>

// CNC Shield X-axis connections
#define STEP_PIN 2
#define DIR_PIN 5
#define ENABLE_PIN 8

// Servo pins
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define CLAW_PIN 11

// Stepper settings
#define STEPS_PER_REV 200
#define MICROSTEPS 16
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)
#define CENTER_STEPS (TOTAL_STEPS / 4)  // 90° position
#define MAX_OFFSET (TOTAL_STEPS / 2)    // ±90° from center

Servo servo1;
Servo servo2;
Servo claw;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int servo1_pos = 90;
int servo2_pos = 90;
int claw_pos = 90;

void setup() {
  Serial.begin(9600);
  
  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  claw.attach(CLAW_PIN);
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  claw.write(claw_pos);
  
  // Initialize stepper
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable driver
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper.setCurrentPosition(CENTER_STEPS);  // Start at 90°
  
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.equals("STOP")) {
      emergencyStop();
    }
    else if (command.startsWith("SERVO")) {
      processServoCommand(command);
    }
    else if (command.startsWith("CLAW")) {
      processClawCommand(command);
    }
    else if (command.startsWith("STEPPER")) {
      processStepperCommand(command);
    }
  }
  
  stepper.run();
}

void emergencyStop() {
  servo1.detach();
  servo2.detach();
  claw.detach();
  digitalWrite(ENABLE_PIN, HIGH);
  stepper.stop();
  stepper.disableOutputs();
  Serial.println("EMERGENCY STOP");
}

void processServoCommand(String command) {
  int space1 = command.indexOf(' ');
  int space2 = command.indexOf(' ', space1 + 1);
  
  if (space1 != -1 && space2 != -1) {
    servo1_pos = command.substring(space1 + 1, space2).toInt();
    servo2_pos = command.substring(space2 + 1).toInt();
    
    servo1_pos = constrain(servo1_pos, 0, 180);
    servo2_pos = constrain(servo2_pos, 0, 180);
    
    servo1.write(servo1_pos);
    servo2.write(servo2_pos);
    
    Serial.print("SERVO ");
    Serial.print(servo1_pos);
    Serial.print(" ");
    Serial.println(servo2_pos);
  }
}

void processClawCommand(String command) {
  int space = command.indexOf(' ');
  if (space != -1) {
    claw_pos = command.substring(space + 1).toInt();
    claw_pos = constrain(claw_pos, 0, 180);
    claw.write(claw_pos);
    Serial.print("CLAW ");
    Serial.println(claw_pos);
  }
}

void processStepperCommand(String command) {
  int space = command.indexOf(' ');
  if (space != -1) {
    long target = command.substring(space + 1).toInt();
    target = constrain(target, CENTER_STEPS - MAX_OFFSET, CENTER_STEPS + MAX_OFFSET);
    stepper.moveTo(target);
    Serial.print("STEPPER ");
    Serial.println(target);
  }
}