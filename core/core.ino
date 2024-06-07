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
int BlueLedPin = 10;
int RedLedPin = 9;

// the following variables were calculated  manually then corroborated by using chat GPT
// using the follwoing parameters on my mower:
// motor RPM: 5RPM
// wheel separation: 20cm.
// Wheel diameter: 17.8cm
// to turn 180 degrees on perfect traction with 17.8 diameters wheels, separated by 20cm at rpm  6.74 seconds is enough
const int TimeFor180DegreeTurn = 7;
const int TimeFor90DegreeTurn = 4; // for a 90 degrees turn would be 3.37 seconds

// fence readers
int FencereaderPinLeft = A7;
int FencereaderPinRight = A6;

// IR sensor
int SensorRunCicle = 0;
bool BatteryProtectionTriggeredStop = false;
bool StoppedBecauseIROutOfRange = false;
int LimitIrDetected = 10;
int IrSensorPinBack = A1;
int IrSensorPinFront = A3;
int IrSensorPinRight = A4;
int IrSensorPinLeft = A5;
int RecommendedDirectionIr = 0;


// bumper buttons
int BumperButtonLeft = 8;
int BumperButtonRight = 4;
volatile byte StateBumperLeft = HIGH;
volatile byte StateBumperRight = HIGH;

// ultrasonic sensors
String DistancesDebug = "---------------";
int UltrasonicSensorLeft =11;
int UltrasonicSensorCenter = 12;
int UltrasonicSensorRight = 7;

// motors
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


// Mode:
int MODE_CHARGING = -2;
int MODE_RESTING = -1;
int MODE_MOWING = 0;
int MODE_RETURNING_HOME = 2;
int MODE_STUCK = 3;

int SUB_MODE_NAVIGATING = 0; // moving fordward
int SUB_MODE_REDIRECTING = 1; // avoiding obstacles
int SUB_MODE_REDIRECTING_TOWARDS_BASE = 2; // Avoids getting too far from the base

int CurrentMode = MODE_MOWING;
int CurrentSubMode = 0;

// batteries:
int VoltageReaderPin = A0;
float PreviousVoltage = 12.6;
float CurrentVoltage = 12.6;
float MarginVoltage = 0.05;
int CurrentBatteryPercentage = 100;
int CicleCounter = 0;

/*
  MOTORS
*/
void SetUpMotorPins(){
  pinMode(MotorPinLeft1, OUTPUT);
  pinMode(MotorPinLeft2, OUTPUT);
  pinMode(MotorPinRight1, OUTPUT);
  pinMode(MotorPinRight2, OUTPUT);
  pinMode(GrassCutterPin, OUTPUT);
}

bool WheelsGoing = false;
void SetStopWheels()//String debug)
{
    //Serial.println("SetStopWheels: "+debug);
    WheelsGoing = false;
    StopCuttingGrass();
    analogWrite(MotorPinLeft1, LOW);
    analogWrite(MotorPinLeft2, LOW);
    analogWrite(MotorPinRight1, LOW);
    analogWrite(MotorPinRight2, LOW);
    delay(3000);
}

void SetMoveLeftMotor(int direction){
  if(direction==1){
    analogWrite(MotorPinLeft1, MotorSpeeds);
    analogWrite(MotorPinLeft2, LOW);
  }else{
    analogWrite(MotorPinLeft1, LOW);
    analogWrite(MotorPinLeft2, MotorSpeeds);
  }
}
void SetMoveRightMotor(int direction){
  if(direction==1){
    analogWrite(MotorPinRight1, MotorSpeeds);
    analogWrite(MotorPinRight2, LOW);
  }else{
    analogWrite(MotorPinRight1, LOW);
    analogWrite(MotorPinRight2, MotorSpeeds);
  }
}

void SetMoveLeft(int time){
  SetMoveLeftMotor(1);
  SetMoveRightMotor(-1);
  MovingWhileDetectedBumperStuck(time);
}
void SetMoveRight(int time){
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(1);
  MovingWhileDetectedBumperStuck(time);
}

void SetMoveFront(){
  WheelsGoing = true;
  SetMoveLeftMotor(1);
  SetMoveRightMotor(1);
}

void SetMoveBackToLocateIR(int time){
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
  delay(1000); // initial delay to help remove preassure
  while(time>0 && !ReadBumperSensors() && ReadIrSensor()==0)
  {
    delay(1000);
    time--;
  }
  if(!ActIfBumperStuck() && ReadIrSensor()>0)
  {
    // it is not stuck and detected the IR sensor:
    SetStopWheels();
  }
}
void SetMoveBack(int time){
  SetMoveLeftMotor(-1);
  SetMoveRightMotor(-1);
  MovingWhileDetectedBumperStuck(time);
}

void MovingWhileDetectedBumperStuck(int time){
  // this function allows side or backwards movement
  // while monitoring the bumpers without interrupts
  delay(1000); // initial delay to help remove preassure from button
  while(time>0 && !ReadBumperSensors())
  {
    delay(1000);
    time--;
  }
  ActIfBumperStuck();
}

boolean ActIfBumperStuck(){
  if(ReadBumperSensors())
  {
    SetMoveFront();
    delay(500); // release button to prevent further damage
    SetStopWheels();
    CurrentMode = MODE_STUCK; // stop mower completely, it is stuck
    return true;
  }
  return false;
}

/*
  END MOTORS
*/

int ReadSingleUltrasonicSensor(int sensor){
  delay(20);
  pinMode(sensor, OUTPUT);
  digitalWrite(sensor, LOW);
  digitalWrite(sensor, HIGH);
  delayMicroseconds(100);
  digitalWrite(sensor, LOW);
  pinMode(sensor, INPUT);
  return pulseIn(sensor, HIGH)* 0.034 /2;
}

int MulticheckUltrasonicDistance(int sensor)
{
  int verifications = 25;
  int distance =  1;
  while(verifications>0)
  {
    distance = max(ReadSingleUltrasonicSensor(sensor), distance);
    verifications--;
  }
  return distance;
}

boolean ReadUltrasonicSensor(){
  DistancesDebug = "Dis:";
  int distanceLeft = MulticheckUltrasonicDistance(UltrasonicSensorLeft);
  int distanceCenter = MulticheckUltrasonicDistance(UltrasonicSensorCenter);
  int distanceRight = MulticheckUltrasonicDistance(UltrasonicSensorRight);
  // Prints the distance on the Serial Monitor
  DistancesDebug = DistancesDebug + distanceLeft+"/";
  DistancesDebug = DistancesDebug + distanceCenter +"/";
  DistancesDebug = DistancesDebug + distanceRight;
  Serial.println(DistancesDebug + " - bat: "+CurrentBatteryPercentage + ". V: "+CurrentVoltage + " - "+ CurrentMode + " Cicle: "+ CicleCounter);

  int distanceAllowed = 35;
  if(CurrentMode == MODE_RETURNING_HOME)
  {
    distanceAllowed =  25;
  }
  boolean collisionDetected = min(distanceLeft, min(distanceCenter, distanceRight)) < distanceAllowed;
  if(collisionDetected)
  {
    if(distanceLeft < distanceRight)
    {
      RecommendedDirection = DirectionRight;
    }else{
      RecommendedDirection = DirectionLeft;
    }
  }
  return collisionDetected;
}

int ReadIrSensor(){
  // in order to consider detected, 2 sensors have to have detected it.
  // if only one sensor has it in range, it is required that the rover does not lose that signal,
  // if only one sensor has it in range, we ask the rover to follow that direction
  int analogIrLeft = analogRead(IrSensorPinLeft);
  int analogIrRight = analogRead(IrSensorPinRight);
  int analogIrBack = analogRead(IrSensorPinBack);
  int analogIrFront = analogRead(IrSensorPinFront);
  int readings = 300;
  int detectedSides = 0;
  int detectedLeft = 0;
  int detectedRight = 0;
  int detectedBack = 0;
  int detectedFront = 0;
  // does multiple readings to make sure all sensors capture data
  // TODO: replace by interrupts, it will remove the need of loop
  while(readings)
  {
    readings--;
    if(analogIrLeft < 900) {
      detectedLeft++;
    }
    if(analogIrRight < 900) {
      detectedRight++;
    }
    if(analogIrBack < 1000)
    {
      detectedBack++;
    }
    if(analogIrFront < 1000)
    {
      detectedFront++;
    }
    analogIrLeft = analogRead(IrSensorPinLeft);
    analogIrRight = analogRead(IrSensorPinRight);
    analogIrBack = analogRead(IrSensorPinBack);
    analogIrFront = analogRead(IrSensorPinFront);
  }
  String debug1 = "Evaluated with [ R: ";
  debug1 = debug1 + detectedRight + " L: - ";
  debug1 =  debug1 + detectedLeft + " B: - ";
  debug1 =  debug1 + detectedBack + "]";
  Serial.println(debug1);
if(detectedBack > 0){
    Serial.println("Detected in the back!");
    RecommendedDirectionIr = DirectionBack;
    RecommendedDirection = DirectionBack;
    detectedSides++;
} else if(detectedLeft>0 && detectedRight>0) {
    // both sensors detect the base signal, so recommends to move fordwardish.
    RecommendedDirectionIr = DirectionCenter;
    RecommendedDirection = DirectionCenter;
    Serial.println("Detected IR Center");
    detectedSides = 2;
  } else if(detectedFront > 0){
    RecommendedDirectionIr = DirectionCenter;
    RecommendedDirection = DirectionCenter;
    Serial.println("Detected IR Front");
    detectedSides++;
  } else if (detectedRight > 0){
    RecommendedDirectionIr = DirectionRight;
    RecommendedDirection = DirectionRight;
    Serial.println("Detected IR Right");
    detectedSides++;
  } else if (detectedLeft > 0){
    RecommendedDirectionIr = DirectionLeft;
    RecommendedDirection = DirectionLeft;
    Serial.println("Detected IR Left");
    detectedSides++;
  } 
  
  return detectedSides;
}

boolean ReadFenceSensors(){
  Serial.println("Evaluating Fence");
  int left = analogRead(FencereaderPinLeft);
  int attempts = 150;
  while (attempts> 0)
  {
    if(left < 30)
    {
      RecommendedDirection = DirectionRight;
      Serial.println("Fence detected Left ");
      // return true;
    }
    left = analogRead(FencereaderPinLeft);
    attempts--;
  }
  int right = analogRead(FencereaderPinRight);
  attempts = 150;
  while (attempts> 0)
  {
    if (right < 30)
    {
      RecommendedDirection = DirectionLeft;
      Serial.println("Fence detected Right ");
      // return true;
    }
    right = analogRead(FencereaderPinRight);
    attempts--;
  }
  Serial.print(left);
  Serial.print(" -- ");
  Serial.print(right);
  Serial.println(" - ");
  return false;
}


// Setup robot microcontroller initial state.
void setup()
{  
  Serial.begin(115200);
  SetUpMotorPins();
  // bumper setup
  pinMode(BumperButtonLeft, INPUT_PULLUP);
  pinMode(BumperButtonRight, INPUT_PULLUP);

  pinMode(IrSensorPinBack, INPUT);
  digitalWrite(IrSensorPinBack, LOW);
  pinMode(IrSensorPinLeft, INPUT);
  pinMode(IrSensorPinRight, INPUT);
  pinMode(RedLedPin, OUTPUT);
  pinMode(BlueLedPin, OUTPUT);
  digitalWrite(BlueLedPin, HIGH);
  digitalWrite(RedLedPin, HIGH);

  CurrentSubMode = SUB_MODE_NAVIGATING;
  CurrentMode = MODE_MOWING;
}

bool ReadBumperSensors(){
  StateBumperLeft = digitalRead(BumperButtonLeft);
  StateBumperRight = digitalRead(BumperButtonRight);
  if(StateBumperLeft == LOW){
    RecommendedDirection = DirectionRight;
    return true;
  }
  if(StateBumperRight == LOW){
    RecommendedDirection = DirectionLeft;
    return true;
  }
  return false;
}

bool DetectCollisionWithSensors(){
  /*
    This is a basic alert from the sensors, if true, sensors are asking to turn arround.
    If false, sensors do not find anything.
  */
  return ReadUltrasonicSensor() || ReadBumperSensors();// || ReadFenceSensors();
}

int GetBatteryPercentage()
{
  if(CurrentVoltage>12.59)
    return 100;
  if(CurrentVoltage>12.44)
    return 95;
  if(CurrentVoltage>12.32)
    return 90;
  if(CurrentVoltage>12.23)
    return 85;
  if(CurrentVoltage>12.05)
    return 80;
  if(CurrentVoltage>11.93)
    return 75;
  if(CurrentVoltage>11.84)
    return 70;
  if(CurrentVoltage>11.72)
    return 65;
  if(CurrentVoltage>11.61)
    return 60;
  if(CurrentVoltage>11.55)
    return 55;
  if(CurrentVoltage>11.52)
    return 50;
  if(CurrentVoltage>11.46)
    return 45;
  if(CurrentVoltage>11.40)
    return 40;
  if(CurrentVoltage>11.37)
    return 35;
  if(CurrentVoltage>11.31)
    return 30;
  if(CurrentVoltage>11.25)
    return 25;
  if(CurrentVoltage>11.19)
    return 20;
  if(CurrentVoltage>11.13)
    return 15;
  if(CurrentVoltage>11.07)
    return 10;
  return 0;
}

int GetBatteryVoltage(){
  // The reason this function is gated to every few cicles is to
  // avoid that sudden spikes or decreases cause the mower to
  // think the battery state is different from what it is
  // for this reason we do the "3 measurement trail".
  // 12.6 = 100
  // v = X
  float R1 = 47000.0;
  float R2 = 10000.0;
  float voltage = analogRead(VoltageReaderPin) * (5.0/1024)*((R1 + R2)/R2);
  float prev = CurrentVoltage;
  CurrentVoltage = ((PreviousVoltage + CurrentVoltage + voltage)/3) + MarginVoltage;
  PreviousVoltage = prev;
  CurrentBatteryPercentage = GetBatteryPercentage();
  return CurrentVoltage;
}

void MoveFordwardABit(int seconds){
  SetMoveFront(); // move front just a bit for better view.
  while(seconds>0)
  {
    if(!DetectCollisionWithSensors())
    {
      delay(1000);
    }
    seconds--;
  }
}


void NavigateAndAvoidObstacles(String objective){
    if (CurrentSubMode == SUB_MODE_REDIRECTING)
    {
      Redirect();
      CutGrass();
    }
    else if(CurrentSubMode == SUB_MODE_REDIRECTING_TOWARDS_BASE)
    {
      // it went too far, it is trying to go closer to home
      if(ReadIrSensor() > 0)
      {
        if (RecommendedDirectionIr == DirectionCenter){
          // base is on target now, continue and call is done
          CurrentSubMode = SUB_MODE_NAVIGATING;
          Serial.println("Transision to SUB_MODE_NAVIGATING");
          SetMoveFront();
        } else if(RecommendedDirectionIr != DirectionNone) {
          MoveToIrRecommendedDirection("SUB_MODE_REDIRECTING_TOWARDS_BASE 1");
          MoveFordwardABit(3);
        } else {
          Serial.println("Recommended direction not found: ");
          Serial.println(RecommendedDirectionIr);
        }
        if(DetectCollisionWithSensors())
        {
          // we redirect here so we can continue on SUB_MODE_REDIRECTING_TOWARDS_BASE
          Redirect();
        }
        CutGrass();
      } else {
        MoveToIrRecommendedDirection("Lost signal. Move towards last known location");
        MoveFordwardABit(3);
        SetStopWheels(); // lost signal of the base, this prevents the scape and prevent to keep collision.
        Serial.println("Lost signal, waiting for one.");
      }
    }
    else if(CurrentSubMode == SUB_MODE_NAVIGATING)
    {
      // Check sensors
      int sensorState = ReadIrSensor();
      if(sensorState==0) // not detected, we have to turn back or try to turn towards it when detected
      {
        CurrentSubMode = SUB_MODE_REDIRECTING_TOWARDS_BASE;
        if(CicleCounter > 10)
        {
          // only move back if has been traveling some time ( # cycles )
          SetMoveBackToLocateIR(10 + CicleCounter);
          Serial.println("Signal Lost, long back track.");
        }
        SetStopWheels();
      }
      else if(sensorState == 1)
      {
        if(RecommendedDirectionIr == DirectionBack)
        {
          // we only have contact in the back, we have to turn around until we have good
          CurrentSubMode = SUB_MODE_REDIRECTING_TOWARDS_BASE;
          MoveToIrRecommendedDirection("Returning to area.");
        } else {
          // we only have contact with one sensor, so we turn to that sensor side
          // it is expected to have good contact with the IR, so more than 1 sensor.
          MoveToIrRecommendedDirection("Turning towards sensor.");
        }
      }
      else if(DetectCollisionWithSensors())
      {
        CurrentSubMode = SUB_MODE_REDIRECTING;
      }
    }
}


void Redirect()
{
    // Collision or redirection
    // Move back enough to not hit anything
    // turn arround
    // time paused depends of the RPM of the motors and how long the mower been stuck
    int upperLimit = 3;
    int lowerLimit =  2;
    if(CurrentSubMode != SUB_MODE_REDIRECTING_TOWARDS_BASE)
    {
      SetMoveBack(random(lowerLimit, upperLimit));
    } else {
      SetMoveBack(1); // barely.
    }
    if(CicleCounter < 10)
    {
      // this means, it recently turned, and might be stuck
      upperLimit = TimeFor180DegreeTurn; // TimeFor180DegreeTurn seconds, gives the chance to turn around a lot more
    }
    int wait_time = random(lowerLimit, upperLimit); // wait a bit
    if(RecommendedDirection == DirectionLeft)
    {
      SetMoveLeft(wait_time);
    } else if(RecommendedDirection == DirectionRight) {
      SetMoveRight(wait_time);
    } else if(RecommendedDirection == DirectionBack)
    {
      SetMoveRight(wait_time*2);
    }
    SetMoveFront();
    // continue for next cicle
    CurrentSubMode = SUB_MODE_NAVIGATING;
    CicleCounter=0;
}

bool MakeSureVoltageIsLessThan(int minimum){
    Serial.println("DC voltage "+ minimum);
    int timesToCheck = 3;
    CicleCounter = 10;
    while (timesToCheck > 0)
    {
      /* code */
      delay(5);
      GetBatteryVoltage();
      CurrentBatteryPercentage = GetBatteryPercentage();
      timesToCheck--;
      if(CurrentBatteryPercentage > minimum)
      {
        return false;
      }
    }
    return true;
}

void SetModeByBatteryPercentage() {
  int MidLimit = 15; // starts looking for charger
  int LowerLimit = 5; // Stop completely
  int chargedAtPercentage = 60; // starts to mow again
  if(CurrentBatteryPercentage >= chargedAtPercentage)
  {
    // battery is charged and ready to work again.
    // ask if can mow
    if(CurrentMode == MODE_CHARGING || CurrentMode == MODE_RETURNING_HOME || CurrentMode == MODE_RESTING) {
      CurrentMode = MODE_MOWING;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      SetMoveFront();
      CutGrass();
    }
    digitalWrite(BlueLedPin, HIGH);
    digitalWrite(RedLedPin, LOW);
  }
  else if(CurrentBatteryPercentage < LowerLimit && MakeSureVoltageIsLessThan(LowerLimit)) {
      if(CurrentMode != MODE_RESTING)
      {
        SetStopWheels();
        delay(3000);
        if(MakeSureVoltageIsLessThan(LowerLimit))
        {
          // Stop moving if battery is lower than x%
          // Stop all action and blink for help
          CurrentMode = MODE_RESTING;
          Serial.println(CicleCounter);
          Serial.println(" - Battery less than 5%: ");
          Serial.println(CurrentBatteryPercentage);
          SetStopWheels();
          BlinkLedPin(RedLedPin, 2);
          BlinkLedPin(BlueLedPin, 2);
        } else {
          SetMoveFront();
          CutGrass();
        }
      }
  }
  else if(CurrentBatteryPercentage < MidLimit  && CurrentMode == MODE_MOWING)
  {
    // look for charging
    if(MakeSureVoltageIsLessThan(MidLimit))
    {
      CurrentMode = MODE_RETURNING_HOME;
      CurrentSubMode = SUB_MODE_NAVIGATING;
      Serial.println("RETURNING HOME");
      Serial.println(CurrentBatteryPercentage);
      digitalWrite(BlueLedPin, LOW);
      digitalWrite(RedLedPin, HIGH);
    }
  }
  else if(CurrentBatteryPercentage < chargedAtPercentage)
  {
    if(CurrentMode == MODE_CHARGING || CurrentMode == MODE_RESTING) {
      BlinkLedPin(RedLedPin, CurrentBatteryPercentage/10);
      delay(1000);
    }
  }
}

void StopCuttingGrass(){  
  digitalWrite(GrassCutterPin, LOW);
}
void CutGrass(){
  digitalWrite(GrassCutterPin, HIGH);
}

bool MoveToIrRecommendedDirection(String debug){
      Serial.println("MoveToIrRecommendedDirection:" + debug);
      // it detects the base:
      if(RecommendedDirectionIr == DirectionLeft)
      {
        SetMoveLeft(TimeFor90DegreeTurn);
        CutGrass();
        Serial.println("Move a bit to the left.");
        RecommendedDirectionIr = DirectionNone;
        return true;
      }
      else if (RecommendedDirectionIr == DirectionRight)
      {
        SetMoveRight(TimeFor90DegreeTurn);
        CutGrass();
        Serial.println("Move a bit to the right.");
        RecommendedDirectionIr = DirectionNone;
        return true;
      }
      else if (RecommendedDirectionIr == DirectionCenter)
      {
        // it seems both sensors are looking the target
        if(ReadUltrasonicSensor() && CurrentMode == MODE_RETURNING_HOME) // it is 10cm at the base
        {
          // Stop moving
          CurrentMode = MODE_CHARGING;
          SetStopWheels();
          Serial.println("Parcked for charging!");
          BlinkLedPin(BlueLedPin, 5);
        }
      }
      else if(RecommendedDirectionIr == DirectionBack){
        SetMoveBack(10);
        SetMoveRight(TimeFor180DegreeTurn);
        Serial.println("Trying to turn back");
        RecommendedDirectionIr = DirectionNone;
        return true;
      } else {
        Serial.println("ERROR: not recommended direction!!!!");
        SetMoveBackToLocateIR(10);
      }
      return false;
}

void BlinkLedPin(int LedPinNumber, int Times){
  int originalState = digitalRead(LedPinNumber);
  while(Times>0) {
    digitalWrite(LedPinNumber, HIGH);
    delay(200);
    digitalWrite(LedPinNumber, LOW);
    delay(200);
    Times--;
  }
  // return to original state
  digitalWrite(LedPinNumber, originalState);
}

void DontLoseIrBaseOfSight(){
  // The goal of this function is to help the rover to turn towards the IR source
  // so it does not lose sight of it
    if(CurrentSubMode == SUB_MODE_NAVIGATING)
    {
      if(ReadIrSensor() > 0)
      {
        // already in sight
        if(RecommendedDirectionIr != DirectionCenter)
        {
          if(MoveToIrRecommendedDirection("DontLoseIrBaseOfSight 1")){
            SetMoveFront();
          }
        }
        if(RecommendedDirectionIr == DirectionBack)
        {
            SetMoveBack(10);
            MoveToIrRecommendedDirection("DontLoseIrBaseOfSight 2");
            SetMoveFront();
        }
      }
    }
}

void loop()
{
  if(CurrentMode == MODE_MOWING) {
     NavigateAndAvoidObstacles("MOWING");
  }
  else if(CurrentMode ==  MODE_RETURNING_HOME)
  {
    NavigateAndAvoidObstacles("RETURNING HOME");
    // Is home detected?
    if(ReadIrSensor() > 0)
    {
      Serial.println("HOME DETECTED");
      MoveToIrRecommendedDirection("HOME DETECTED");
    }
  }
  else if(CurrentMode == MODE_RESTING)
  {
    SetStopWheels();
    digitalWrite(BlueLedPin, LOW);
    digitalWrite(RedLedPin, LOW);
    BlinkLedPin(RedLedPin, 3);
    String deb1 = CicleCounter+" Resting Mode, battery at:" + CurrentBatteryPercentage;
    Serial.println(deb1);
  }
  else if(CurrentMode == MODE_CHARGING)
  {
    SetStopWheels();
    BlinkLedPin(RedLedPin, CurrentBatteryPercentage/10);
    Serial.println("Low Battery - Charging");
    SensorRunCicle = 0;
  }
  else if(CurrentMode == MODE_STUCK){
    Serial.println("Unknown error/state flash all leds");
    SetStopWheels();
    /// Unknown error/state flash all leds:
    digitalWrite(RedLedPin, HIGH);
    digitalWrite(BlueLedPin, HIGH);
    delay(2000);
    digitalWrite(RedLedPin, LOW);
    digitalWrite(BlueLedPin, LOW);
  }
  if(CicleCounter > 0 && CicleCounter%30 == 0)
  {
    GetBatteryVoltage();
    SetModeByBatteryPercentage();
  }
  if(CicleCounter > 0  && CicleCounter%15 == 0)
  {
    DontLoseIrBaseOfSight();
  }
  if(GetBatteryVoltage() < 10)
  { 
    BatteryProtectionTriggeredStop = WheelsGoing;
    SetStopWheels();
    BlinkLedPin(RedLedPin, 2);
    Serial.println("Battery protection triggered");
  } else if (BatteryProtectionTriggeredStop) {
    BatteryProtectionTriggeredStop = false;
    SetMoveFront();
  }
  SensorRunCicle++;
  CicleCounter++;
  // ReadFenceSensors();
  // delay(2000);
  // if(CicleCounter>15)
  // {
  //   SetMoveFront();
  // }
}
