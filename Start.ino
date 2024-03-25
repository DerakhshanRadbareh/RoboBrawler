
#define MOTOR_LEFT_PIN_1 10
#define MOTOR_LEFT_PIN_2 11
#define MOTOR_RIGHT_PIN_1 9
#define MOTOR_RIGHT_PIN_2 6

// Threshold infrared sensors that based on the calibrated value
#define lineThreshold 800

// Servo pin definition (PWM pin)
#define servoPin 5

// Initial left and right speed of analog PWM servo motors
byte speedL = 150;
byte speedR = 160;

// Define gripper positions
#define positionRightPulseWidth 2000   // About 2 ms as micro servo 9g - SG90 datasheet
#define positionMiddlePulseWidth 1650  // About 1.5 ms as micro servo 9g - SG90 datasheet
#define positionLeftPulseWidth 950     // About 1 ms as micro servo 9g - SG90 datasheet

// Set analog pins to infrared sensors
int irSensors[] = { A0, A1, A2, A3, A4, A5, A6, A7 };

// Initial infrared values
int irValues[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

// Value to check if robot in line after move left IR
boolean isInLine = false;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_2, OUTPUT);

  // Set servo pin as output
  pinMode(servoPin, OUTPUT);

  // Timer setting for 50 Hz frequency
  TCCR1A = (1 << WGM11) | (1 << WGM10);
  TCCR1B = (1 << CS11);
  OCR1A = 20000;  // 20,000 microseconds for a 50 Hz period

  // Initial gripper position
  openGripper();

  countLine(4);
  delay(300);

  closeGripper();
  delay(200);

  moveOutOfSquare();

  moveLeftIR();
  delay(200);

  straightForward();
}

void loop() {
}

// Functions to control robot move to forward
void moveForward(byte leftSpeedf, byte rightSpeed) {
  analogWrite(MOTOR_RIGHT_PIN_1, 0);
  analogWrite(MOTOR_RIGHT_PIN_2, rightSpeed);
  analogWrite(MOTOR_LEFT_PIN_1, 0);
  analogWrite(MOTOR_LEFT_PIN_2, leftSpeedf);

  delay(10);
}

// Functions to stop robot
void stopDrive() {
  analogWrite(MOTOR_RIGHT_PIN_1, 0);
  analogWrite(MOTOR_RIGHT_PIN_2, 0);
  analogWrite(MOTOR_LEFT_PIN_1, 0);
  analogWrite(MOTOR_LEFT_PIN_2, 0);
}

// Functions to control robot move to forward
void moveLeftIR() {
  boolean lineDetected = false;

  analogWrite(MOTOR_RIGHT_PIN_1, 0);
  analogWrite(MOTOR_RIGHT_PIN_2, 149);
  analogWrite(MOTOR_LEFT_PIN_1, 255);
  analogWrite(MOTOR_LEFT_PIN_2, 0);

  while (!lineDetected) {
    // Read sensor values
    irValues[0] = analogRead(irSensors[0]);
    irValues[1] = analogRead(irSensors[1]);
    irValues[2] = analogRead(irSensors[2]);
    irValues[3] = analogRead(irSensors[3]);
    irValues[4] = analogRead(irSensors[4]);
    irValues[5] = analogRead(irSensors[5]);
    irValues[6] = analogRead(irSensors[6]);
    irValues[7] = analogRead(irSensors[7]);

    if (irValues[3] > lineThreshold || irValues[4] > lineThreshold) {
      lineDetected = true;
    }
  }

  stopDrive();
}

void straightForward() {
  while (true) {
    irValues[0] = analogRead(irSensors[0]);
    irValues[1] = analogRead(irSensors[1]);
    irValues[2] = analogRead(irSensors[2]);
    irValues[3] = analogRead(irSensors[3]);
    irValues[4] = analogRead(irSensors[4]);
    irValues[5] = analogRead(irSensors[5]);
    irValues[6] = analogRead(irSensors[6]);
    irValues[7] = analogRead(irSensors[7]);

    if (irValues[3] < lineThreshold && irValues[4] < lineThreshold) {
      if (!isInLine) {
        moveForward(200, 160);
      } else {
        stopDrive();
        break;
      }
    } else if (irValues[3] > lineThreshold && irValues[4] > lineThreshold) {
      moveForward(150, 160);
      isInLine = true;
    } else if (irValues[3] < lineThreshold && irValues[4] > lineThreshold) {
      moveForward(200, 160);
      isInLine = true;
    } else if (irValues[3] > lineThreshold && irValues[4] < lineThreshold) {
      moveForward(150, 200);
      isInLine = true;
    }
  }
}

// Servo position adjustment function
void setServoPosition(int pulseWidth) {
  // Send PWM pulse to servo
  int i = 1;
  while (i <= 10) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
    i++;
  }
}

// Gripper opening function
void openGripper() {
  setServoPosition(positionMiddlePulseWidth);
}

// Gripper closing function
void closeGripper() {
  setServoPosition(positionLeftPulseWidth);
}

// Count black lines and move forward until finished black line counted
void countLine(int target) {
  int lineCount = 0;
  boolean lineDetected = false;

  moveForward(speedL, speedR);

  while (lineCount < target) {
    // Read sensor values
    irValues[0] = analogRead(irSensors[0]);
    irValues[1] = analogRead(irSensors[1]);
    irValues[2] = analogRead(irSensors[2]);
    irValues[3] = analogRead(irSensors[3]);
    irValues[4] = analogRead(irSensors[4]);
    irValues[5] = analogRead(irSensors[5]);
    irValues[6] = analogRead(irSensors[6]);
    irValues[7] = analogRead(irSensors[7]);

    // Toggle between checking for line vs. checking for no line
    if (lineDetected == false) {
      // if all sensors detect a line, increase line count and toggle to checking for no line
      if (irValues[1] > lineThreshold && irValues[2] > lineThreshold && irValues[3] > lineThreshold && irValues[4] > lineThreshold && irValues[5] > lineThreshold && irValues[6] > lineThreshold) {
        lineCount++;
        lineDetected = true;
      }
    } else {
      // if all sensors detect no line, toggle back to checking for line
      if (irValues[1] < lineThreshold && irValues[2] < lineThreshold && irValues[3] < lineThreshold && irValues[4] < lineThreshold && irValues[5] < lineThreshold && irValues[6] < lineThreshold) {
        lineDetected = false;
      }
    }
  }

  stopDrive();
}

void moveOutOfSquare() {
  boolean isOutsideSquare = false;
  moveForward(speedL, speedR);

  while (!isOutsideSquare) {
    // Read sensor values
    irValues[0] = analogRead(irSensors[0]);
    irValues[1] = analogRead(irSensors[1]);
    irValues[2] = analogRead(irSensors[2]);
    irValues[3] = analogRead(irSensors[3]);
    irValues[4] = analogRead(irSensors[4]);
    irValues[5] = analogRead(irSensors[5]);
    irValues[6] = analogRead(irSensors[6]);
    irValues[7] = analogRead(irSensors[7]);

    if (irValues[2] < lineThreshold && irValues[3] < lineThreshold && irValues[4] < lineThreshold) {
      isOutsideSquare = true;
    }
  }

  // stopDrive();
}
