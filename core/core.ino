#include <Arduino.h>

/*
Pins used:
A0: Voltage
A1: IrSensorPinBack
A2: GrassCutterPin
A3: IrSensorPinFront
A4: IrSensorPinLeft
A5: IrSensorPinRight
A6: FencereaderPinLeft
A7: FencereaderPinRight
D2: MotorPinRight2
D3: MotorPinRight1
D4: BumperButtonRight
D5: MotorPinLeft2
D6: MotorPinLeft1
D7: UltrasonicSensorRight
D8: BumperButtonLeft
D9: BlueLedPin
D10: RedLedPin
D11: UltrasonicSensorLeft
D12: UltrasonicSensorCenter
D13: FREE
*/

// Light Indicators
int BlueLedPin = 9;
int RedLedPin = 10;

// Motor parameters
const int TimeFor180DegreeTurn = 6; // Time for a 180-degree turn
const int TimeFor90DegreeTurn = 3;  // Time for a 90-degree turn

// Fence readers
int FencereaderPinLeft = A7;
int FencereaderPinRight = A6;

// IR sensor
int SensorRunCycle = 0;
bool BatteryProtectionTriggeredStop = false;
bool StoppedBecauseIROutOfRange = false;
int LimitIrDetected = 10;
int IrSensorPinBack = A1;
int IrSensorPinFront = A3;
int IrSensorPinRight = A4;
int IrSensorPinLeft = A5;
int RecommendedDirectionIr = 0;

// Array to store IR sensor values
int IrSensorValues[4] = {0, 0, 0, 0};
// Enumeration for array indices
enum { LEFT, RIGHT, BACK, FRONT };

// How many sides the IR was detected from
int IRDetectedSides = 0;

// Bumper buttons
int BumperButtonLeft = 8;
int BumperButtonRight = 4;
volatile byte StateBumperLeft = HIGH;
volatile byte StateBumperRight = HIGH;

// Ultrasonic sensors
String DistancesDebug = "---------------";
int UltrasonicSensorLeft = 11;
int UltrasonicSensorCenter = 12;
int UltrasonicSensorRight = 7;

// Motors
int GrassCutterPin = A2;
int DirectionNone = -1;
int DirectionRight = 1;
int DirectionLeft = 2;
int DirectionCenter = 3;
int DirectionBack = 4;
int RecommendedDirection = 0;
int MotorSpeeds = 255;

int MotorPinRight1 = 3;
int MotorPinRight2 = 2;
int MotorPinLeft1 = 6;
int MotorPinLeft2 = 5;

// Mode definitions
int MODE_CHARGING = -2;
int MODE_RESTING = -1;
int MODE_MOWING = 0;
int MODE_RETURNING_HOME = 2;
int MODE_STUCK = 3;

int SUB_MODE_NAVIGATING = 0; // Moving forward
int SUB_MODE_REDIRECTING = 1; // Avoiding obstacles
int SUB_MODE_REDIRECTING_TOWARDS_BASE = 2; // Redirecting towards base

int CurrentMode = MODE_MOWING;
int CurrentSubMode = 0;

// Batteries
int VoltageReaderPin = A0;
float PreviousVoltage = 12.6;
float CurrentVoltage = 12.6;
float MarginVoltage = 0.05;
int CurrentBatteryPercentage = 100;
int CycleCounter = 0;

/*
  MOTORS
*/
void SetUpMotorPins() {
  pinMode(MotorPinLeft1, OUTPUT);
  pinMode(MotorPinLeft2, OUTPUT);
  pinMode(MotorPinRight1, OUTPUT);
  pinMode(MotorPinRight2, OUTPUT);
  pinMode(GrassCutterPin, OUTPUT);
}

bool WheelsGoing = false;

void SetStopWheels(String debug) {
  Serial.println("SetStopWheels: " + debug);
  WheelsGoing = false;
  StopCuttingGrass();
  analogWrite(MotorPinLeft1, LOW);
  analogWrite(MotorPinLeft2, LOW);
  analogWrite(MotorPinRight1, LOW);
  analogWrite(MotorPinRight2, LOW);
  delay(3000);
}

void SetMoveLeftMotor(int direction) {
  if (direction == 1) {
    analogWrite(MotorPinLeft1, MotorSpeeds);
    analogWrite(MotorPinLeft2, LOW);
  } else {
    analogWrite(MotorPinLeft1, LOW);
    analogWrite(MotorPinLeft2, MotorSpeeds);
  }
}

void SetMoveRightMotor(int direction) {
  if (direction == 1) {
    analogWrite(MotorPinRight1, MotorSpeeds);
    analogWrite(MotorPinRight2, LOW);
  } else {
    analogWrite(MotorPinRight1, LOW);
    analogWrite(MotorPinRight2, MotorSpeeds);
  }
}

void SetMoveLeft(int time) {
  WheelsGoing = true;
  SetMoveLeftMotor(1);
  SetMoveRightMotor(-1);
  MovingWhileDetectedBumperStuck(time);
}

void SetMoveRight(int time) {
  WheelsGoing = true;
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(1);
  MovingWhileDetectedBumperStuck(time);
}

void SetMoveFront() {
  if (!ReadBumperSensors()) {
    WheelsGoing = true;
    SetMoveLeftMotor(1);
    SetMoveRightMotor(1);
  } else {
    Serial.println("Collision detected, can't move front.");
  }
}

void SetMoveBackToLocateIR(int time) {
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
  delay(1000); // Initial delay to help remove pressure
  time = time * 2;
  while (time > 0 && !ReadBumperSensors() && ReadIrSensor() == 0) {
    delay(500);
    time--;
  }
  if (!ActIfBumperStuck() && ReadIrSensor() > 0) {
    // It is not stuck and detected the IR sensor:
    SetStopWheels("ActIfBumperStuck");
  }
}

void SetMoveBack(int time) {
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
  MovingWhileDetectedBumperStuck(time);
  CycleCounter = 0;
}

void MovingWhileDetectedBumperStuck(int time) {
  // This function allows side or backwards movement
  // while monitoring the bumpers without interrupts
  delay(2500); // Initial delay to help remove pressure from button
  time = time * 2;
  while (time > 0 && !ReadBumperSensors() && ReadIrSensor() > 2) {
    delay(500);
    time--;
  }
  ActIfBumperStuck();
}

boolean ActIfBumperStuck() {
  if (ReadBumperSensors()) {
    SetMoveFront();
    delay(500); // Release button to prevent further damage
    SetStopWheels("ActIfBumperStuck");
    CurrentMode = MODE_STUCK; // Stop mower completely, it is stuck
    return true;
  }
  return false;
}

/*
  END MOTORS
*/

int ReadSingleUltrasonicSensor(int sensor) {
  pinMode(sensor, OUTPUT);
  digitalWrite(sensor, LOW);
  digitalWrite(sensor, HIGH);
  delayMicroseconds(100);
  digitalWrite(sensor, LOW);
  pinMode(sensor, INPUT);
  return pulseIn(sensor, HIGH) * 0.034 / 2;
}

int MulticheckUltrasonicDistance(int sensor) {
  /*
    The sensors seem not to be 100% reliable,
    so we perform multiple tests, and always get the max number.
  */
  int verifications = 25;
  int distance = 1;
  while (verifications > 0) {
    distance = max(ReadSingleUltrasonicSensor(sensor), distance);
    verifications--;
  }
  return min(distance, 500);
}

boolean ReadUltrasonicSensor() {
  return ReadUltrasonicSensor(25);
}

boolean ReadUltrasonicSensor(int distanceAllowed) {
  DistancesDebug = "Dis:";
  int distanceLeft = MulticheckUltrasonicDistance(UltrasonicSensorLeft);
  int distanceCenter = MulticheckUltrasonicDistance(UltrasonicSensorCenter);
  int distanceRight = MulticheckUltrasonicDistance(UltrasonicSensorRight);

  // Prints the distance on the Serial Monitor
  DistancesDebug = DistancesDebug + distanceLeft + "/";
  DistancesDebug = DistancesDebug + distanceCenter + "/";
  DistancesDebug = DistancesDebug + distanceRight;
  Serial.println(DistancesDebug + " - bat: " + CurrentBatteryPercentage + ". V: " + CurrentVoltage + " - " + CurrentMode + " Cycle: " + CycleCounter);

  boolean collisionDetected = min(distanceLeft, min(distanceCenter, distanceRight)) < distanceAllowed;
  if (collisionDetected) {
    if (distanceLeft < distanceRight) {
      RecommendedDirection = DirectionRight;
    } else {
      RecommendedDirection = DirectionLeft;
    }
  }
  return collisionDetected;
}

void debugIrSensors(int detectedRight, int detectedLeft, int detectedBack, int detectedFront) {
  Serial.print("IR [ R: ");
  Serial.print(detectedRight);
  Serial.print(" L: ");
  Serial.print(detectedLeft);
  Serial.print(" B: ");
  Serial.print(detectedBack);
  Serial.print(" F: ");
  Serial.print(detectedFront);
  Serial.println("]");
}

int ReadIrSensor() {
  // In order to consider detected, 2 sensors have to have detected it.
  // If only one sensor has it in range, it is required that the rover does not lose that signal,
  // if only one sensor has it in range, we ask the rover to follow that direction
  int analogIrLeft = analogRead(IrSensorPinLeft);
  int analogIrRight = analogRead(IrSensorPinRight);
  int analogIrBack = analogRead(IrSensorPinBack);
  int analogIrFront = analogRead(IrSensorPinFront);
  int readings = 300;
  int threshold = 1; // Readings detected, to filter false positives

  // Does multiple readings to make sure all sensors capture data
  // TODO: Replace with interrupts, it will remove the need for loop
  IrSensorValues[LEFT] = 0;
  IrSensorValues[BACK] = 0;
  IrSensorValues[FRONT] = 0;
  IrSensorValues[RIGHT] = 0;
  IRDetectedSides = 0;
  while (readings) {
    readings--;
    if (analogIrLeft < 800) {
      IrSensorValues[LEFT]++;
    }
    if (analogIrRight < 800) {
      IrSensorValues[RIGHT]++;
    }
    if (analogIrBack < 800) {
      IrSensorValues[BACK]++;
    }
    if (analogIrFront < 800) {
      IrSensorValues[FRONT]++;
    }
    analogIrLeft = analogRead(IrSensorPinLeft);
    analogIrRight = analogRead(IrSensorPinRight);
    analogIrBack = analogRead(IrSensorPinBack);
    analogIrFront = analogRead(IrSensorPinFront);
  }
  debugIrSensors(IrSensorValues[RIGHT], IrSensorValues[LEFT], IrSensorValues[BACK], IrSensorValues[FRONT]);

  if (IrSensorValues[LEFT] > threshold && IrSensorValues[RIGHT] > threshold && IrSensorValues[FRONT] > threshold) {
    // Both sensors detect the base signal, so recommends moving forward.
    RecommendedDirectionIr = DirectionCenter;
    RecommendedDirection = DirectionCenter;
    IRDetectedSides = 3;
  } else if (IrSensorValues[FRONT] > threshold) {
    RecommendedDirectionIr = DirectionCenter;
    RecommendedDirection = DirectionCenter;
    IRDetectedSides++;
  } else if (IrSensorValues[RIGHT] > threshold) {
    RecommendedDirectionIr = DirectionRight;
    RecommendedDirection = DirectionRight;
    IRDetectedSides++;
  } else if (IrSensorValues[LEFT] > threshold) {
    RecommendedDirectionIr = DirectionLeft;
    RecommendedDirection = DirectionLeft;
    IRDetectedSides++;
  }

  if (IrSensorValues[BACK] > threshold) {
    if (IRDetectedSides == 0) {
      Serial.println("Detected in the back ONLY!");
      // Only detected in the back
      RecommendedDirectionIr = DirectionBack;
      RecommendedDirection = DirectionBack;
    }
    IRDetectedSides++;
  }
  
  return IRDetectedSides;
}

// Function to smooth analog readings
int smoothAnalogRead(int pin) {
  int totalReadings = 1;
  int total = 0;
  for (int i = 0; i < totalReadings; i++) { // Average readings
    total += analogRead(pin);
  }
  return total / totalReadings;
}

const int boundaryCalibrationAttempts = 100; // Number of readings during calibration
// Other calibration variables
int baseLeftReading = 0;
int baseRightReading = 0;
int thresholdReadings = 97; // 755; // Adjust based on actual readings

void SetUpAndCalibrateBoundaryValues() {
  /*
  It is assumed the lawnmower starts on top of the boundary wire. 
  It will take readings and set up the values to detect the wire,
  then it will move backwards and set the values for when the wire is not detected.
  */
  int leftSum = 0;
  int rightSum = 0;
  
  // SetMoveBack to include the motors in the readings
  CutGrass();
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
  
  // Take readings directly over the boundary wire
  for (int i = 0; i < boundaryCalibrationAttempts; i++) {
    leftSum += smoothAnalogRead(FencereaderPinLeft);
    rightSum += smoothAnalogRead(FencereaderPinRight);
    delay(10); // Small delay between readings
  }
  
  baseLeftReading = leftSum / boundaryCalibrationAttempts;
  baseRightReading = rightSum / boundaryCalibrationAttempts;
  
  Serial.println("Done calibration part 1.");
  SetMoveBack(50);
  SetMoveLeft(TimeFor180DegreeTurn);
  Serial.println("Starts calibration part 2.");
  
  leftSum = 0;
  rightSum = 0;
  
  for (int i = 0; i < boundaryCalibrationAttempts; i++) {
    leftSum += smoothAnalogRead(FencereaderPinLeft);
    rightSum += smoothAnalogRead(FencereaderPinRight);
    delay(10); // Small delay between readings
  }
  
  int outLeftReading = leftSum / boundaryCalibrationAttempts;
  int outRightReading = rightSum / boundaryCalibrationAttempts;
  
  // Calculate threshold as a midpoint between in-boundary and out-boundary readings
  thresholdReadings = (baseLeftReading + baseRightReading + outLeftReading + outRightReading) / 4;
  
  SetStopWheels("Done calibration part 2.");
  Serial.print("Base Left: ");
  Serial.print(baseLeftReading);
  Serial.print(" | Base Right: ");
  Serial.print(baseRightReading);
  Serial.print(" | Out Left: ");
  Serial.print(outLeftReading);
  Serial.print(" | Out Right: ");
  Serial.print(outRightReading);
  Serial.print(" | Threshold: ");
  Serial.println(thresholdReadings);
  Serial.println("Starts in... 5s.");
  delay(5000);
}

const int thresHoldLowerTimes = 4; // Number of minimum positive detections needed
int thresHoldUpperTimes = 100; // Upper limit of positive detections (noise)
const int attempts = 300; // Number of attempts for detection

boolean ReadFenceSensors() {
  int leftPositives = 0;
  int rightPositives = 0;
  int averageReadingL = 0;
  int averageReadingR = 0;

  // Read left sensor
  for (int i = 0; i < attempts; i++) {
    int left = smoothAnalogRead(FencereaderPinLeft);
    averageReadingL = averageReadingL + left;
    if (left < thresholdReadings) {
      leftPositives++;
    }
  }

  // Read right sensor
  for (int i = 0; i < attempts; i++) {
    int right = smoothAnalogRead(FencereaderPinRight);
    averageReadingR = averageReadingR + right;
    if (right < thresholdReadings) {
      rightPositives++;
    }
  }

  // Determine recommended direction
  if (leftPositives > rightPositives) {
    RecommendedDirection = DirectionLeft;
  } else {
    RecommendedDirection = DirectionRight;
  }

  // Print results for debugging
  int avrL = (averageReadingL / attempts);
  int avrR = (averageReadingR / attempts);
  Serial.print("Av L: ");
  Serial.print(avrL);
  Serial.print("Av R: ");
  Serial.print(avrR);
  Serial.print("Le Pos: ");
  Serial.print(leftPositives);
  Serial.print(" | Ri Pos: ");
  Serial.println(rightPositives);

  // Return true if either sensor detected enough positives
  return (thresHoldUpperTimes > leftPositives && leftPositives > thresHoldLowerTimes) || (thresHoldUpperTimes > rightPositives && rightPositives > thresHoldLowerTimes);
}

void CalibrationOfBoundaryWire() {
  delay(10000);
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  Serial.println("Initializing readings for calibration");
  SetUpAndCalibrateBoundaryValues();
  Serial.println("Finalized calibration. Starts in 5s.");
  delay(5000);
}

// Setup robot microcontroller initial state.
void setup() {  
  Serial.begin(9600);
  SetUpMotorPins();
  // Bumper setup
  pinMode(BumperButtonLeft, INPUT_PULLUP);
  pinMode(BumperButtonRight, INPUT_PULLUP);

  pinMode(FencereaderPinLeft, INPUT);
  pinMode(FencereaderPinRight, INPUT);

  pinMode(IrSensorPinBack, INPUT);
  pinMode(IrSensorPinLeft, INPUT);
  pinMode(IrSensorPinRight, INPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  digitalWrite(BlueLedPin, HIGH);
  digitalWrite(RedLedPin, HIGH);

  // CalibrationOfBoundaryWire();
  CurrentSubMode = SUB_MODE_NAVIGATING;
  CurrentMode = MODE_MOWING;
}

bool ReadBumperSensors() {
  StateBumperLeft = digitalRead(BumperButtonLeft); // Due to hardware available bumper switch is different.
  StateBumperRight = digitalRead(BumperButtonRight);
  if (StateBumperLeft == HIGH) {
    RecommendedDirection = DirectionRight;
    return true;
  }
  if (StateBumperRight == LOW) {
    RecommendedDirection = DirectionLeft;
    return true;
  }
  return false;
}

bool DetectCollisionWithSensors() {
  /*
    This is a basic alert from the sensors, if true, sensors are asking to turn around.
    If false, sensors do not find anything.
  */
  return ReadUltrasonicSensor() || ReadBumperSensors(); // || ReadFenceSensors();
}

int GetBatteryPercentage() {
  if (CurrentVoltage > 12.59)
    return 100;
  if (CurrentVoltage > 12.44)
    return 95;
  if (CurrentVoltage > 12.32)
    return 90;
  if (CurrentVoltage > 12.23)
    return 85;
  if (CurrentVoltage > 12.05)
    return 80;
  if (CurrentVoltage > 11.93)
    return 75;
  if (CurrentVoltage > 11.84)
    return 70;
  if (CurrentVoltage > 11.72)
    return 65;
  if (CurrentVoltage > 11.61)
    return 60;
  if (CurrentVoltage > 11.55)
    return 55;
  if (CurrentVoltage > 11.52)
    return 50;
  if (CurrentVoltage > 11.46)
    return 45;
  if (CurrentVoltage > 11.40)
    return 40;
  if (CurrentVoltage > 11.37)
    return 35;
  if (CurrentVoltage > 11.31)
    return 30;
  if (CurrentVoltage > 11.25)
    return 25;
  if (CurrentVoltage > 11.19)
    return 20;
  if (CurrentVoltage > 11.13)
    return 15;
  if (CurrentVoltage > 11.07)
    return 10;
  return 0;
}

int GetBatteryVoltage() {
  // The reason this function is gated to every few cycles is to
  // avoid that sudden spikes or decreases cause the mower to
  // think the battery state is different from what it is
  // for this reason we do the "3 measurement trail".
  // 12.6 = 100
  // v = X
  float R1 = 47000.0;
  float R2 = 10000.0;
  float voltage = analogRead(VoltageReaderPin) * (5.0 / 1024) * ((R1 + R2) / R2);
  float prev = CurrentVoltage;
  CurrentVoltage = ((PreviousVoltage + CurrentVoltage + voltage) / 3) + MarginVoltage;
  PreviousVoltage = prev;
  CurrentBatteryPercentage = GetBatteryPercentage();
  return CurrentVoltage;
}

void TurnTheOppositeOfTheIR() {
  // This attempts request the IR to turn the opposite direction of the IR
  // but always having 2 sensors detecting it
  // this is a very custom-made code, since the IR is at the edge of my garden
  DetectCollisionWithSensors(); // Reads recommended direction to turn
  int tryCounter = 20; // Attempts limited to avoid permanent loop
  if (RecommendedDirection == DirectionLeft) {
    ReadIrSensor();
    while (IrSensorValues[BACK] < 10 && IRDetectedSides > 1 && tryCounter > 0) {
      SetMoveBack(1);
      SetMoveLeft(TimeFor90DegreeTurn);
      SetStopWheels("TurnTheOppositeOfTheIR 90Turn");
      ReadIrSensor();
      tryCounter--;
    }
  } else {
    ReadIrSensor();
    while (IrSensorValues[BACK] < 10 && IRDetectedSides > 1 && tryCounter > 0) {
      SetMoveBack(1);
      SetMoveRight(TimeFor90DegreeTurn);
      SetStopWheels("TurnTheOppositeOfTheIR 90Turn");
      ReadIrSensor();
      tryCounter--;
    }
  }
  SetMoveFront();
  CutGrass();
}

void ProtectBattery() {
  if (GetBatteryVoltage() < 10) { 
    BatteryProtectionTriggeredStop = WheelsGoing;
    if (WheelsGoing && IRDetectedSides > 2) {
      // More than 2 sides on IR detected, otherwise we might get lost
      SetMoveBack(20);
    }
    SetStopWheels("Battery Protection.");
    BlinkLedPin(RedLedPin, 2);
  } else if (BatteryProtectionTriggeredStop) {
    BatteryProtectionTriggeredStop = false;
    if (IRDetectedSides > 2) {
      // When battery protection is triggered is usually when
      // colliding with something near the IR base, this is very custom to my lawn/garden.
      // But sometimes it is triggered when it is going uphill too, so this prevents from getting lost.
      TurnTheOppositeOfTheIR();
      SetMoveFront();
    } else if (IRDetectedSides == 2) {
      MoveToIrRecommendedDirection("ProtectBattery.");
      SetMoveFront();
    } else {
      CurrentSubMode == SUB_MODE_REDIRECTING;
    }
  }
}

void ProtectBumpers() {
  if (WheelsGoing && (ReadBumperSensors() || ReadUltrasonicSensor(15))) {
    SetMoveBack(15);
    delay(500);
    SetStopWheels("Emergency stop ReadBumperSensors");
    TurnTheOppositeOfTheIR();
  }
}

bool DetectWireBoundary() {
  if (WheelsGoing) {
    int positiveDetection = 0;
    int reads = 10;
    int confirmedTimes = 0;
    int confirmationValue = 4;
    bool detected = ReadFenceSensors();
    if (detected) {
      // If the first detection worked, then multiple polling should help to confirm:
      // eventually multiplying the total polling by more than a 1000 times.
      SetStopWheels("Stopped wheels to confirm detection of boundary wire");
      while (reads > 0 && confirmedTimes < confirmationValue) {
        detected = ReadFenceSensors();
        if (detected) {
          confirmedTimes++;
        }
        reads--;
      }
      if (confirmedTimes >= confirmationValue) {
        Serial.println("Detected Wire Fence");
        SetMoveBack(5);
        Redirect();
        return true;
      }
    }
  }
  return false;
}

void MoveForwardABit(int seconds) {
  SetMoveFront(); // Move front just a bit for better view.
  seconds = seconds * 2;
  while (seconds > 0) {
    if (!DetectCollisionWithSensors()) {
      delay(500);
    } else {
      SetStopWheels("Move forward a bit - Collision.");
    }
    seconds--;
  }
}

void NavigateAndAvoidObstacles(String objective) {
  Serial.println(objective);
  if (CurrentSubMode == SUB_MODE_REDIRECTING) {
    int irSensorState = ReadIrSensor();
    if (irSensorState > 2 && RecommendedDirection == DirectionCenter) {
      // Close to the IR base, turn back to navigate away
      SetMoveRight(TimeFor180DegreeTurn);
    } else {
      Redirect();
    }
    CutGrass();
  } else if (CurrentSubMode == SUB_MODE_REDIRECTING_TOWARDS_BASE) {
    // It went too far, it is trying to go closer to home
    if (ReadIrSensor() > 0) {
      if (RecommendedDirectionIr == DirectionCenter) {
        // Base is on target now, continue and call is done
        CurrentSubMode = SUB_MODE_NAVIGATING;
        Serial.println("Transition to SUB_MODE_NAVIGATING");
      } else if (RecommendedDirectionIr != DirectionNone) {
        MoveToIrRecommendedDirection("SUB_MODE_REDIRECTING_TOWARDS_BASE 1");
      } else {
        Serial.println("Recommended direction not found.");
      }
      MoveForwardABit(1);
      CutGrass();
    } else if (WheelsGoing) {
      MoveToIrRecommendedDirection("Move towards last known location");
      SetStopWheels("Lost signal of the base"); // Lost signal of the base, this prevents escape and prevent to keep collision.
      Serial.println("Lost signal, waiting for one.");
    } else {
      SetMoveLeft(2);
      SetStopWheels("Searching for base");
    }
    if (WheelsGoing && DetectCollisionWithSensors()) {
      // We redirect here so we can continue on SUB_MODE_REDIRECTING_TOWARDS_BASE
      CurrentSubMode = SUB_MODE_REDIRECTING; // TODO: Testing
    }
  } else if (CurrentSubMode == SUB_MODE_NAVIGATING) {
    // Check sensors
    int irSensorState = ReadIrSensor();
    if (irSensorState == 0) { // Not detected, we have to turn back or try to turn towards it when detected
      CurrentSubMode = SUB_MODE_REDIRECTING_TOWARDS_BASE;
      if (CycleCounter > 10 && WheelsGoing) {
        // Only move back if has been traveling some time ( # cycles )
        SetMoveBackToLocateIR(10 + CycleCounter);
        Serial.println("Signal Lost, long back track.");
      }
      SetMoveBackToLocateIR(4);
      SetStopWheels("Signal Lost, long back track (?).");
    } else if (irSensorState == 1) {
      if (RecommendedDirectionIr == DirectionBack) {
        // We only have contact in the back, we have to turn around until we have good
        CurrentSubMode = SUB_MODE_REDIRECTING_TOWARDS_BASE;
        MoveToIrRecommendedDirection("Returning to area.");
      } else if (RecommendedDirectionIr == DirectionCenter) {
        // We only have contact with one sensor, so we turn to that sensor side
        // It is expected to have good contact with the IR, so more than 1 sensor.
        MoveToIrRecommendedDirection("Turning towards sensor. Only detected by one sensor.");
      }
      SetMoveFront();
    }
    if (DetectCollisionWithSensors()) {
      CurrentSubMode = SUB_MODE_REDIRECTING;
    }
  }
}

bool bumperCollision = false; // TODO: Move

void Redirect() {
  // Collision or redirection
  // Move back enough to not hit anything
  // Turn around
  // Time paused depends on the RPM of the motors and how long the mower has been stuck
  int upperLimit = 6;
  int lowerLimit = 2;
  int wait_time = random(lowerLimit, upperLimit); // Wait a bit
  int preventStuck = 10;
  if (CurrentSubMode != SUB_MODE_REDIRECTING_TOWARDS_BASE || bumperCollision) {
    SetMoveBack(random(lowerLimit, upperLimit));
    wait_time = TimeFor180DegreeTurn;
  } else {
    SetMoveBack(2); // Barely.
  }
  if (CycleCounter < 10) {
    // This means it recently turned and might be stuck
    upperLimit = TimeFor180DegreeTurn; // TimeFor180DegreeTurn seconds, gives the chance to turn around a lot more
  }
  if (RecommendedDirection == DirectionLeft) {
    SetMoveLeft(TimeFor90DegreeTurn);
    while (ReadUltrasonicSensor() && preventStuck > 0) {
      SetMoveLeft(1);
      SetStopWheels("Moving and reading distances");
      preventStuck--;
    }
  } else if (RecommendedDirection == DirectionRight) {
    SetMoveRight(TimeFor90DegreeTurn);
    while (ReadUltrasonicSensor() && preventStuck > 0) {
      SetMoveRight(1);
      SetStopWheels("Moving and reading distances");
      preventStuck--;
    }
  } else if (RecommendedDirection == DirectionBack) {
    SetMoveRight(TimeFor180DegreeTurn);
  }
  MoveForwardABit(1);
  // Continue for next cycle
  CurrentSubMode = SUB_MODE_NAVIGATING;
  CycleCounter = 0;
  bumperCollision = false;
}

bool MakeSureVoltageIsLessThan(int minimum) {
  int timesToCheck = 3;
  CycleCounter = 10;
  while (timesToCheck > 0) {
    delay(5);
    GetBatteryVoltage();
    CurrentBatteryPercentage = GetBatteryPercentage();
    timesToCheck--;
    if (CurrentBatteryPercentage > minimum) {
      return false;
    }
  }
  return true;
}

void SetModeByBatteryPercentage() {
  int MidLimit = 15; // Starts looking for charger
  int LowerLimit = 5; // Stop completely
  int chargedAtPercentage = 60; // Starts to mow again

  if (CurrentBatteryPercentage >= chargedAtPercentage) {
    // Battery is charged and ready to work again.
    // Ask if it can mow
    if (CurrentMode == MODE_CHARGING || CurrentMode == MODE_RETURNING_HOME || CurrentMode == MODE_RESTING) {
      CurrentMode = MODE_MOWING;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      SetMoveFront();
      CutGrass();
    }
    digitalWrite(BlueLedPin, HIGH);
    digitalWrite(RedLedPin, LOW);
  } else if (CurrentBatteryPercentage < LowerLimit && MakeSureVoltageIsLessThan(LowerLimit)) {
    if (CurrentMode != MODE_RESTING) {
      SetStopWheels("Measure battery");
      delay(3000);
      if (MakeSureVoltageIsLessThan(LowerLimit)) {
        // Stop moving if battery is lower than x%
        // Stop all action and blink for help
        CurrentMode = MODE_RESTING;
        Serial.println(CycleCounter + " - Battery less than 5%: ");
        SetStopWheels("Low battery: " + CurrentBatteryPercentage);
        BlinkLedPin(RedLedPin, 2);
        BlinkLedPin(BlueLedPin, 2);
      } else {
        SetMoveFront();
        CutGrass();
      }
    }
  } else if (CurrentBatteryPercentage < MidLimit && CurrentMode == MODE_MOWING) {
    // Look for charging
    if (MakeSureVoltageIsLessThan(MidLimit)) {
      CurrentMode = MODE_RETURNING_HOME;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      Serial.println(CurrentBatteryPercentage + "% BATT. RETURNING HOME");
      digitalWrite(BlueLedPin, LOW);
      digitalWrite(RedLedPin, HIGH);
    }
  } else if (CurrentBatteryPercentage < chargedAtPercentage) {
    if (CurrentMode == MODE_CHARGING || CurrentMode == MODE_RESTING) {
      SetStopWheels("MODE_RESTING - MODE_CHARGING");
      BlinkLedPin(RedLedPin, CurrentBatteryPercentage / 10);
      delay(1000);
    }
  }
}

void StopCuttingGrass() {  
  digitalWrite(GrassCutterPin, LOW);
}

void CutGrass() {
  digitalWrite(GrassCutterPin, HIGH);
}

bool MoveToIrRecommendedDirection(String debug) {
  Serial.println("MoveToIrRecommendedDirection: " + debug);
  // It detects the base:
  if (RecommendedDirectionIr == DirectionLeft) {
    SetMoveLeft(TimeFor90DegreeTurn);
    CutGrass();
    Serial.println("Move a bit to the left.");
    RecommendedDirectionIr = DirectionNone;
    return true;
  } else if (RecommendedDirectionIr == DirectionRight) {
    SetMoveRight(TimeFor90DegreeTurn);
    CutGrass();
    Serial.println("Move a bit to the right.");
    RecommendedDirectionIr = DirectionNone;
    return true;
  } else if (RecommendedDirectionIr == DirectionCenter) {
    // It seems both sensors are looking at the target
    if (ReadUltrasonicSensor() && CurrentMode == MODE_RETURNING_HOME) { // It is 10cm at the base
      // Stop moving
      CurrentMode = MODE_CHARGING;
      SetStopWheels("MODE_CHARGING");
      Serial.println("Parked for charging!");
      BlinkLedPin(BlueLedPin, 5);
    }
  } else if (RecommendedDirectionIr == DirectionBack) {
    SetMoveBack(10);
    SetMoveRight(TimeFor180DegreeTurn);
    Serial.println("Recommended direction is back: Trying to turn back");
    RecommendedDirectionIr = DirectionNone;
    return true;
  } else {
    Serial.println("ERROR: Not recommended direction!!!!");
    SetMoveBackToLocateIR(10);
  }
  return false;
}

void BlinkLedPin(int LedPinNumber, int Times) {
  int originalState = digitalRead(LedPinNumber);
  while (Times > 0) {
    digitalWrite(LedPinNumber, HIGH);
    delay(200);
    digitalWrite(LedPinNumber, LOW);
    delay(200);
    Times--;
  }
  // Return to original state
  digitalWrite(LedPinNumber, originalState);
}

void DontLoseIrBaseOfSight() {
  // The goal of this function is to help the rover to turn towards the IR source
  // so it does not lose sight of it
  if (CurrentSubMode == SUB_MODE_NAVIGATING) {
    int irReading = ReadIrSensor();
    if (irReading > 0) {
      // Already in sight
      if (RecommendedDirectionIr != DirectionCenter && irReading <= 2) {
        if (MoveToIrRecommendedDirection("DontLoseIrBaseOfSight 1")) {
          MoveForwardABit(1);
        }
      } else if (RecommendedDirectionIr == DirectionBack) {
        SetMoveBack(10);
        MoveToIrRecommendedDirection("DontLoseIrBaseOfSight 2");
        MoveForwardABit(1);
      } else if (RecommendedDirectionIr == DirectionCenter && irReading > 1) {
        // The front sensor has a good view of the base, so this will attempt to make it explore in the opposing way.
        SetMoveRight(TimeFor180DegreeTurn);
      }
    }
  }
}

void AttemptToRecover() {
  // Check all the sensors and attempt to keep working:
  if (ReadIrSensor() > 0) {
    if (ReadBumperSensors() || ReadUltrasonicSensor()) {
      SetMoveBack(20);
      SetStopWheels("AttemptToRecover Stop");
    } else {
      if (CurrentBatteryPercentage > 20) {
        SetMoveBack(3);
        SetMoveRightMotor(TimeFor180DegreeTurn);
        SetMoveFront();
        CurrentMode = MODE_MOWING;
        CurrentSubMode = SUB_MODE_NAVIGATING;
      } else {
        Serial.println("Battery a bit low to continue working");
      }
    }
  }
}

void loop() {
  Serial.print("CycleCounter: ");
  Serial.println(CycleCounter);
  if (CurrentMode == MODE_MOWING) {
    NavigateAndAvoidObstacles("MOWING");
    int irsensor = ReadIrSensor();
    if (WheelsGoing == false && !DetectCollisionWithSensors() && irsensor > 0 && !ReadFenceSensors()) {
      if (RecommendedDirectionIr == DirectionCenter && irsensor >= 3) {
        Serial.println("Attempts to turn around since it seems too south.");
        SetMoveBack(5);
        SetMoveLeft(TimeFor180DegreeTurn);
        SetMoveFront();
      } else {
        // It was stopped, but we attempt to keep it moving
        Serial.println("Move forward since MODE_MOWING.");
        SetMoveFront();
      }
    } else if (WheelsGoing == false && DetectCollisionWithSensors()) {  
      SetMoveBack(10);
      SetStopWheels("Release ReadBumperSensors");
    }
    CutGrass();
  } else if (CurrentMode == MODE_RETURNING_HOME) {
    NavigateAndAvoidObstacles("RETURNING HOME");
    // Is home detected?
    if (ReadIrSensor() > 0 && !DetectCollisionWithSensors()) {
      Serial.println("HOME DETECTED");
      MoveToIrRecommendedDirection("HOME DETECTED");
      SetMoveFront();
    } else {
      SetStopWheels("No IR detected when returning home.");
    }
  } else if (CurrentMode == MODE_RESTING) {
    SetStopWheels("MODE_RESTING");
    Serial.println("Battery at:" + CurrentBatteryPercentage);
    digitalWrite(BlueLedPin, LOW);
    digitalWrite(RedLedPin, LOW);
    BlinkLedPin(RedLedPin, 3);
  } else if (CurrentMode == MODE_CHARGING) {
    SetStopWheels("MODE_CHARGING");
    Serial.println("Battery at:" + CurrentBatteryPercentage);
    BlinkLedPin(RedLedPin, CurrentBatteryPercentage / 10);
    Serial.println("Low Battery - Charging");
    SensorRunCycle = 0;
  } else if (CurrentMode == MODE_STUCK) {
    SetStopWheels("Unknown error/state flash all LEDs");
    // Unknown error/state flash all LEDs:
    digitalWrite(RedLedPin, HIGH);
    digitalWrite(BlueLedPin, HIGH);
    delay(2000);
    digitalWrite(RedLedPin, LOW);
    digitalWrite(BlueLedPin, LOW);
    AttemptToRecover();
  }
  if ((CycleCounter > 0 && CycleCounter % 100 == 0) || !WheelsGoing) {
    GetBatteryVoltage();
    SetModeByBatteryPercentage();
    if (WheelsGoing) {
      DontLoseIrBaseOfSight();
      Serial.println("Don't lose IR out of sight.");
    }
  }

  ProtectBattery();
  ProtectBumpers();
  DetectWireBoundary();
  SensorRunCycle++;
  CycleCounter++;
}
