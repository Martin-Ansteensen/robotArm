#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// For uploading new code select arduino nano and old bootloarder, avrisp mkll

#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

// Defines the servos range of motion
#define TORSO_SERVOMIN  120 // Left or right?
#define TORSO_SERVOMAX  610 // Left or right?

#define SHOULDER_SERVOMIN  170 // Right
#define SHOULDER_SERVOMAX  580 // Left

#define ELBOW_SERVOMIN  140 // Left
#define ELBOW_SERVOMAX  590 // Right

#define WRIST_SERVOMIN  200 // Right 
#define WRIST_SERVOMAX  660 // Left

#define HAND_SERVOMIN  120 // Does not matter
#define HAND_SERVOMAX  610 // Does not matter

#define GRIPPER_SERVOMAX 400 // Open
#define GRIPPER_SERVOMIN 140 // Closed

// Declaring variables
char *armServos[6] = {"GRIPPER_", "HAND_", "WRIST_", "ELBOW_", "SHOULDER_", "TORSO_"}; // The name of each servo
int minServoLimits[6] = {TORSO_SERVOMIN, SHOULDER_SERVOMIN, ELBOW_SERVOMIN, WRIST_SERVOMIN, HAND_SERVOMIN, GRIPPER_SERVOMIN};
int maxServoLimits[6] = {TORSO_SERVOMAX, SHOULDER_SERVOMAX, ELBOW_SERVOMAX, WRIST_SERVOMAX, HAND_SERVOMAX, GRIPPER_SERVOMAX};
int servoPos[6] = {0, 0, 0, 0, 0, 0}; // The servos positions
int servoPastPos[6] = {0, 0, 0, 0, 0, 0}; // The servos previous position

int potPins[6] = {0, 1 ,2 ,3 ,6 ,7};

int pot0Value;
int pot1Value;
int pot2Value;
int pot3Value;
int pot4Value;
int pot5Value;

int saved0[20];
int saved1[20];
int saved2[20];
int saved3[20];
int saved4[20];
int saved5[20];

int positionArray[6];

void setup() {
  // Console
  Serial.begin(9600);
  Serial.println("Robot arm running!");

  // Servo
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
  
  // Pots
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  pinMode(6, OUTPUT); // LED
  digitalWrite(13, LOW);
  
  delay(10);

  // Initialize all servos
  Shutdown(); 
  MoveJoint(3, 90, 10);
  MoveJoint(2, 90, 20);
  MoveJoint(1, 90, 30);
  MoveJoint(0, 90, 20);
  saved0[0] = -1;
}
 
void loop() {
  if (digitalRead(8)==0){
    int recorded = 0;
    while (digitalRead(8)==0){
      Serial.println("Controlling by pots and recodring positions");
      ControlByPots();
      if (digitalRead(7)==0){
        saved0[recorded] = servoPastPos[0];
        saved1[recorded] = servoPastPos[1];
        saved2[recorded] = servoPastPos[2];
        saved3[recorded] = servoPastPos[3];
        saved4[recorded] = servoPastPos[4];
        saved5[recorded] = servoPastPos[5];
        Serial.println("");
        Serial.println(recorded);        
        Serial.println("saved positions");
        digitalWrite(6, HIGH);
        delay(1000);
        digitalWrite(6, LOW);
        recorded++;
      }
    }
    saved0[recorded] = -1;
  } else if (digitalRead(8)==1 and digitalRead(9)==1){
    Serial.println("Looping trough recorded positions");
    for (int i = 0; i < 20; i++){
      if (saved0[i]==-1){
        Serial.println(i);
        Serial.println(saved0[i]);
        break;
      }
      MoveMultipleJoints(ConvertToArray(saved0[i],saved1[i],saved2[i],saved3[i],saved4[i],saved5[i]), 0);
    }
    
  } else { // Play preprogrammed
    
    Serial.println("Playing preprogrammed positions");
    if (digitalRead(7)==0){
      digitalWrite(6, HIGH);
      Shutdown();
      delay(8000);
    }
    MoveMultipleJoints(ConvertToArray(0, 90, 90, 180, 180, 180), 0);
    delay(2000);
    MoveMultipleJoints(ConvertToArray(0, 90, 90, 0, 0, 0), 0);
    delay(2000);
  }
}

int ConvertToArray(int servo0, int servo1, int servo2, int servo3, int servo4, int servo5){
  positionArray[0] = servo0;
  positionArray[1] = servo1;
  positionArray[2] = servo2;
  positionArray[3] = servo3;
  positionArray[4] = servo4;
  positionArray[5] = servo5;
  return positionArray;
}

/**********************************************************
 * Before the power to the arm is cut the servos should go to a speceif postion.
 * By setting these postions as the first when powering it up we avoid the servos going really fast 
 * to a random postion that we don't know where is
**********************************************************/
void Shutdown() {
  Serial.println("Shutting down");
  MoveJoint(5, 0, 10);
  MoveJoint(4, 0, 10);
  MoveJoint(3, 180, 10);
  MoveJoint(2, 0, 30);
  MoveJoint(1, 0, 30);
  MoveJoint(0, 180, 15);
}

// Moves given joint to position
void MoveJoint(int servo, int degree, int acceleration) {

  // If the previous positions is smaller than the new one (move towards 180 degrees)
  if (servoPastPos[servo] <= degree) {
    for (servoPos[servo]; servoPos[servo] <= degree; servoPos[servo]++) {
      pwm.setPWM(servo, 0, AngleToPulse(servoPos[servo], servo));
      delay(acceleration); // The delay wont be visible to the naked eye, it will only look like the servo is moving slower
    }
  }

  // If the previous positions is larger than the new one (move towards 0 degrees)
  else if (servoPastPos[servo] >= degree) {
    for (servoPos[servo]; servoPos[servo] >= degree; servoPos[servo]--) {
      pwm.setPWM(servo, 0, AngleToPulse(servoPos[servo], servo));
      delay(acceleration); // The delay wont be visible to the naked eye, it will only look like the servo is moving slower
    }
  }
  
  servoPastPos[servo] = servoPos[servo]; // Sets the past position of the servo equal to the current
}

void MoveMultipleJoints(int a[], int acceleration) {
  int degForServos[6] = {a[0], a[1], a[2], a[3], a[4], a[5]}; // Stores the inout in an array
  Serial.println("MoveMultiple Joints Input");
  // for(int i=0; i<6; i++){
  //   Serial.println(a[i]);
  // }
  // Serial.println("");
  for (int deg=0; deg <= 180; deg++) {
    for (int servo=0; servo < 6; servo++) {
      if (servoPastPos[servo] < deg) {
        pwm.setPWM(servo, 0, AngleToPulse(map(deg, 0, 180, servoPastPos[servo], degForServos[servo]), servo));
      }
      
      else if (servoPastPos[servo] > deg) {
        pwm.setPWM(servo, 0, AngleToPulse(map(deg, 0, 180, servoPastPos[servo], degForServos[servo]), servo)); //maybe switch 180 and 0
      }
      
    }
  }
  // Sets the past position of the servo equal to the current
  for (int servo; servo < 6; servo++) {
    servoPastPos[servo] = servoPos[servo] = degForServos[servo];
  } 
}

void ControlByPots(){
  Serial.println("Control By Pots Input");
  for (int i = 0; i<6; i++){
    int deg = map(analogRead(potPins[i]), 0, 1024, 0, 180);
    MoveJoint(i, deg, 0);
    // Serial.println(deg);
  }
  // delay(1000);
}

// Converts pulselength to angle
int AngleToPulse(int ang, int servo) {
  int pulse = map(ang, 0, 180, minServoLimits[servo], maxServoLimits[servo]);
  return pulse;
}
