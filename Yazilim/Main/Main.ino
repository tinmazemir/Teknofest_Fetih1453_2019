#include <Servo.h>
#include <PPMReader.h>
#include <InterruptHandler.h>
#define DISPX 620
#define DISPY 480

#define BMODEDISTANCE 250 // backMOde acikken geri gitmek icin kullanacagi alani kontrol edecek
#define SLOPETOLEFTDISTANCE 300
#define SLOPETORIGHTDISTANCE 300
#define SCANFINISHEDGEDISTANCE 150
#define LRSCANPARALLELISM 40 // sol sag sensor
#define BSCANPARALLESISM 15

#define PULLUPVALUE 160
#define PULLDOWNVALUE 20

int scanShip(int backMode, int scanMode );

Servo MOTOR1;
Servo MOTOR2;
Servo MOTOR3;
Servo MOTOR4;
Servo SERVOMOTOR1;
Servo SERVOMOTOR2;
Servo SERVOMOTOR3;
Servo SERVOMOTOR4;
Servo FIREMOTOR1;
Servo FIREMOTOR2;
Servo RIGHTARM;
Servo LEFTARM;

int motor1speed = 0;
int motor2speed = 0;
int motor3speed = 0;
int motor4speed = -5;
int fireSpeed = 0;
int rightArmAngle = 0;
int leftArmAngle = 0;
int maxStraightSpeed = 40;
int maxSideSpeed = 40;
int maxTurnSpeed = 180;

int targetx = DISPX / 2;
int interruptPin = 3;
int channelAmount = 8;
int RCValues[8] = {
  0, 0, 0, 0, 0, 0, 0, 0};

int sensorPins[6] = {
  A8, A9, A10, A11, A12, A14};
int sensorOfsets[6] = {
  0, 0, 0, 0, 0, 0};

int servoState[] = {
  1, 1, 1, 1};
int pullTimes[] = {
  2000, 2000, 2000, 2000};
int firstTime = 0;
int firstTime1 = 0;
int preValue = 0;
int switchValue = 0;
unsigned long servo1Start;
unsigned long servo2Start;
unsigned long servo3Start;
unsigned long servo4Start;

int mainState = 9;
int dropShipState = 0;

float sObstacle_5 = 0 ;
float sObstacle_4 = 0;
float sObstacle_3 = 0;
float sObstacle_2 = 0;
float sObstacle_1 = 0;
float sObstacleL_0 = 0;
int sObstacleF = 0;

int pidSpeedDist;
int pidSpeedPar;

PPMReader ppm(interruptPin, channelAmount);

int objectX = 0 ;
int objectY = 0;
int buff[8];
bool startSignal = false;

void RCControl() {
  motor1speed = 0;
  motor2speed = 0;
  motor3speed = 0;
  motor4speed = -5;
  int fireSpeed = 0;
  int ch1 = RCValues[0];
  int ch2 = RCValues[1];
  int ch3 = RCValues[2];
  int ch4 = RCValues[3];
  int ch5 = RCValues[4];

  if (ch2 < 1480) {
    motor1speed = motor1speed + map(ch2, 1480, 1000, 0, maxStraightSpeed);
    motor3speed = motor3speed + map(ch2, 1480, 1000, 0, maxStraightSpeed);
  }
  else if (ch2 > 1510) {
    motor2speed = motor2speed + map(ch2, 1510, 2000, 0, maxStraightSpeed);
    motor4speed = motor4speed + map(ch2, 1510, 2000, 0, maxStraightSpeed);
  }

  if (ch1 > 1480) {
    motor1speed = motor1speed + map(ch1, 1480, 1976, 0, maxSideSpeed);
    motor2speed = motor2speed + map(ch1, 1480, 1976, 0, maxSideSpeed);
  }
  else if (ch1 < 1468) {
    motor3speed = motor3speed + map(ch1, 1468, 1000, 0, maxSideSpeed);
    motor4speed = motor4speed + map(ch1, 1468, 1000, 0, maxSideSpeed);
  }

  if (ch4 < 1474) {
    motor1speed = motor1speed + map(ch4, 1474, 1000, 0, maxTurnSpeed);
    motor4speed = motor4speed + map(ch4, 1474, 1000, 0, maxTurnSpeed);
  }
  else if (ch4 > 1490) {
    motor3speed = motor3speed + map(ch4, 1490, 1984, 0, maxTurnSpeed);
    motor2speed = motor2speed + map(ch4, 1490, 1984, 0, maxTurnSpeed);
  }
  if (ch3 > 1200) {
    fireSpeed = map(ch3, 1200, 2000, 0, 120);
  }
  else {
    fireSpeed = 0;
  }
  rightArmAngle = map(RCValues[7], 1000, 2000, 180, 0);
  leftArmAngle = map(RCValues[5], 1000, 2000, 180, 0);
  if (RCValues[4] < 1300) {
    switchValue = 0;
  }
  else {
    switchValue = 1;
  }
  if (switchValue != preValue) {
    firstTime = 0;
    firstTime1 = 0;
    int fIsOK = 0;
    int bIsOK = 0;
    do {
      fIsOK = FrontMotors(switchValue);
      bIsOK = BackMotors(switchValue);
    } 
    while (fIsOK == 0 || bIsOK == 0);
  }
  preValue = switchValue;

  if (ch1 != 0 && ch2 != 0 && ch4 != 0 && ch5 != 0) {
    if (motor1speed > 100)
      motor1speed = 100;
    if (motor2speed > 100)
      motor2speed = 100;
    if (motor3speed > 100)
      motor3speed = 100;
    if (motor4speed > 100)
      motor4speed = 100;
    if (servoState[0] == 1 && servoState[1] == 1 && servoState[2] == 1 && servoState[3] == 1) {
      MOTOR1.write(motor1speed);
      MOTOR2.write(motor2speed);
      MOTOR3.write(motor3speed);
      MOTOR4.write(motor4speed);
      //      Serial.print(motor1speed);
      //      Serial.print(" ");
      //      Serial.print(motor2speed);
      //      Serial.print(" ");
      //      Serial.print(motor3speed);
      //      Serial.print(" ");
      //      Serial.println(motor4speed);
    }
    //FIREMOTOR1.write(fireSpeed);
    //FIREMOTOR2.write(fireSpeed);
    //RIGHTARM.write(rightArmAngle);
    //LEFTARM.write(leftArmAngle);
  }
}

void takeData(char color){
  char data;
  int counter = 0;
  if(Serial.available())
  {
    data = Serial.read();
    if(data == color)
    {
      for(int i = 0; i < 8;)
      {
        if(Serial.available())
        {
          buff[i] = data;
          data = Serial.read();
          i++;
        }
      }
      objectX = 0;
      objectY = 0;
      for(int i = 1; i < 8; i++)
      {
        switch (i)
        {
        case 1:
          objectX += (buff[i]-48)*100;
          break;
        case 2:
          objectX += (buff[i]-48)*10;
          break;
        case 3:
          objectX += buff[i]-48;
          break;
        case 5:
          objectY += (buff[i]-48)*100;
          break;
        case 6:
          objectY += (buff[i]-48)*10;
          break;
        case 7:
          objectY += buff[i]-48;
          break;
        }
      }
    }
  }
}

int FrontMotors(int order) {
  MOTOR1.write(0);
  MOTOR2.write(0);
  MOTOR3.write(0);
  MOTOR4.write(0);
  if (firstTime == 0) {
    servo1Start = millis();
    servo3Start = millis();
  }
  firstTime = 1;
  if (servoState[0] != order) {
    if (millis() - servo1Start < pullTimes[0]) {
      if (order == 1) {
        SERVOMOTOR1.write(180);
      }
      else if (order == 0) {
        SERVOMOTOR1.write(0);
      }
    }
    else {
      SERVOMOTOR1.write(90);
      servoState[0] = order;
    }
  }
  if (servoState[2] != order) {
    if (millis() - servo3Start < pullTimes[2]) {
      if (order == 1) {
        SERVOMOTOR3.write(PULLDOWNVALUE);
      }
      else if (order == 0) {
        SERVOMOTOR3.write(PULLUPVALUE);
      }
    }
    else {
      SERVOMOTOR3.write(90);
      servoState[2] = order;
    }
  }
  if (servoState[0] == order && servoState[2] == order) {
    return 1;
  }
  else {
    return 0;
  }
}

int BackMotors(int order) {
  MOTOR1.write(0);
  MOTOR2.write(0);
  MOTOR3.write(0);
  MOTOR4.write(0);
  if (firstTime1 == 0) {
    servo2Start = millis();
    servo4Start = millis();
  }
  firstTime1 = 1;
  if (servoState[1] != order) {
    if (millis() - servo2Start < pullTimes[1]) {
      if (order == 1) {
        SERVOMOTOR2.write(PULLDOWNVALUE);
      }
      else if (order == 0) {
        SERVOMOTOR2.write(PULLUPVALUE);
      }
    }
    else {
      SERVOMOTOR2.write(90);
      servoState[1] = order;
    }
  }
  if (servoState[3] != order) {
    if (millis() - servo4Start < pullTimes[3]) {
      if (order == 1) {
        SERVOMOTOR4.write(PULLUPVALUE);
      }
      else if (order == 0) {
        SERVOMOTOR4.write(PULLDOWNVALUE);
      }
    }
    else {
      SERVOMOTOR4.write(90);
      servoState[3] = order;
    }
  }
  if (servoState[1] == order && servoState[3] == order) {
    return 1;
  }
  else {
    return 0;
  }
}

int sensorValue(int index) {
  long int sensorValue = 0;
  for (int i = 0; i < 300; i++) {
    switch (index) {
    case 0:
      sensorValue += analogRead(A10);
      break;
    case 1:
      sensorValue += analogRead(A11);
      break;
    case 2:
      sensorValue += analogRead(A9);
      break;
    case 3:
      sensorValue += analogRead(A12);
      break;
    case 4:
      sensorValue += analogRead(A13);
      break;
    case 5:
      sensorValue += analogRead(A8);
      break;
    case 6:
      sensorValue = analogRead(A14);
      break;
    }
  }
  if (index != 6) {
    int dis = sensorValue / 300;

    if (dis >= 280 && dis <= 512)
    {
      int distance = (28250 / (dis - 229.5)) + sensorOfsets[index];
      return distance;
    }
    else {
      return 0;
    }
  }
  else {
    if (sensorValue > 500)
      return 0;
    else
      return 1;
  }
}


double mapfloat(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double pid(double Setpoint, double inp) {
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error;
  double lastError;
  double output;
  double cumError, rateError;
  double kp = mapfloat(RCValues[5], 1000, 2000, 0, 2);
  double kd = mapfloat(RCValues[7], 1000, 2000, 0, 2);
  double ki = 0;
  //Serial.println(RCValues[5]);
  //Serial.print(kp);
  //Serial.print(" ");
  //Serial.println(kd);
  //Serial.print("- -");
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  error = Setpoint - inp;                                // determine error7
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative
  double out = kp * error + ki * cumError + kd * rateError;          //PID output
  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time
  return out;                                        //have function return the PID output
}

void drive(int motor1speed, int motor2speed, int motor3speed, int motor4speed) { //Her motorun hiz degerleri parametre olarak alinir ve surulur.
  if (motor1speed > maxTurnSpeed)
  {
    motor1speed = maxTurnSpeed;
  }
  if (motor2speed > maxTurnSpeed)
  {
    motor2speed = maxTurnSpeed;
  }
  if (motor3speed > maxTurnSpeed)
  {
    motor3speed = maxTurnSpeed;
  }
  if (motor4speed > maxTurnSpeed)
  {
    motor4speed = maxTurnSpeed;
  }

  MOTOR1.write(motor1speed);
  MOTOR2.write(motor2speed);
  MOTOR3.write(motor3speed);
  MOTOR4.write(motor4speed);
}

void backPar(int target) {
  motor1speed = 0;
  motor2speed = 0;
  motor3speed = 0;
  motor4speed = -5;
  int leftFront = sensorValue(0);
  int leftBack = sensorValue(1);
  //int backLeft = sensorValue(2);
  //int backRight = sensorValue(3);
  //int rightBack = sensorValue(4);
  //int rightFront = sensorValue(5);

  int error = leftFront - leftBack;
  //int errorRight = rightBack - rightFront;
  //int totalError = (errorLeft + errorRight)/2;
  //Serial.println(error);
  //Serial.print("  ");
  //Serial.print(leftFront);
  //Serial.print("  ");
  //Serial.println(leftBack);
  //int error = backLeft - backRight;
  int preErr = error;
  double pidSpeed = 0;
  if (1 == 1) {
    //Serial.println(error);
    if (error > 10 || error < -10) {
      pidSpeed = pid(target, error);
      //Serial.print(" Pid Speed: ");
      Serial.println(pidSpeed);
      if (pidSpeed < 0) {
        //drive(-pidSpeed, 0, 0, -pidSpeed);
        motor1speed = motor1speed - pidSpeed;
        //motor4speed = motor4speed - pidSpeed;
      }
      else if (pidSpeed > 0) {
        //drive(0, pidSpeed, pidSpeed, 0);
        //motor2speed = motor2speed + pidSpeed;
        motor3speed = motor3speed + pidSpeed;
      }
      else{
        Serial.println("fak");
      }
    }
    else
    {
      //Serial.println("stop");
    }
  }
  else{
    Serial.println("---------------");
  }
  drive(motor1speed, motor2speed, motor3speed, motor4speed);
}

void takeShip(){
  motor1speed = 0;
  motor2speed = 0;
  motor3speed = 0;
  motor4speed = 0;
  int shipError = 310 - objectX;
  int pidSpeed = 0;
  pidSpeed = pid(0,shipError);
  Serial.print("shipError ::");
  Serial.println(shipError);
  if(shipError < -10){
    motor1speed = motor1speed + pidSpeed;
    motor2speed = motor2speed + pidSpeed;
    Serial.println("solllll");

  }
  else if(shipError > 10){
    motor3speed = motor3speed -pidSpeed;
    motor4speed = motor4speed -pidSpeed;
    Serial.println("sagggggg");

  }
  drive(motor1speed,motor2speed,motor3speed,motor4speed);
}


int target = 0;
int firstTime3 =1;
int preErr = 0;

void startPos() {
  MOTOR1.write(0);
  MOTOR2.write(0);
  MOTOR3.write(0);
  MOTOR4.write(0);
  LEFTARM.write(95);
  RIGHTARM.write(102);
}


void setup() {
  MOTOR1.attach(6, 1000, 2000);
  MOTOR2.attach(7, 1000, 2000);
  MOTOR3.attach(44, 1000, 2000);
  MOTOR4.attach(5, 1000, 2000);
  SERVOMOTOR1.attach(11);
  SERVOMOTOR2.attach(12);
  SERVOMOTOR3.attach(46);
  SERVOMOTOR4.attach(10);
  FIREMOTOR1.attach(2, 1000, 2000);
  FIREMOTOR2.attach(4, 1000, 2000);

  RIGHTARM.attach(9);
  LEFTARM.attach(13);
  Serial.begin(115200);

  // while(!startSignal)
  // {
  //    int data = Serial.read();
  //    if(data == 's')
  //    {
  //       startSignal = true;
  //       Serial.println("Start");
  //    }
  // }
  //startPos();
}
int justStarted = 0;
void loop() {
  while(justStarted < 1){
    startPos();
    justStarted += 1;
    delay(400);
  }
  for (int channel = 1; channel <= channelAmount; ++channel) {
    unsigned long value = ppm.latestValidChannelValue(channel, 0);
    RCValues[channel - 1] = value;
  }
  if (RCValues[6] < 1500) {
    //RCControl();
    RIGHTARM.write(122);
    LEFTARM.write(72);
  }
  else {
    RIGHTARM.write(102);
    LEFTARM.write(95);
    //RIGHTARM.write(80);
    //LEFTARM.write(100);
    //takeData('b');
    //Serial.print(objectX);
    //Serial.print(" ");
    //Serial.println(objectY);
    //takeShip();

  }
}
