#include <L298N.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

MPU6050 mpu6050(Wire);

float errorangle;
float desiredangle = 0.00;
float kp = 1.00;

int CL;
int CR;
int scl = A5;
int sda = A4;
int ENA = 10;
int ENB = 11;
int IN1 = 9;
int IN2 = 10;
int IN3 = 12;
int IN4 = 11;
int Trig = 4;
int Echo = 3;
int buzzer = 6;
long duration, distance;
const int IN_A0 = A0; // analog input
const int IN_D0 = 7; // digital input


Servo servo_motor; 

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
// enconder
  
  
  servo_motor.attach(5); //our servo pin
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  analogWrite(ENA, HIGH);
  analogWrite(ENB, HIGH);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);

  servo_motor.write(115);
  delay(2000);
  }
void loop() {
// put your main code here, to run repeatedly:

 digitalWrite(buzzer, LOW);
 digitalWrite(Trig, HIGH);
  //delayMicroseconds(2);
  delay(100);
  digitalWrite(Trig, LOW);
  //delayMicroseconds(10);
  duration = pulseIn(Echo, HIGH);
  distance = duration * 0.017;
  Serial.println(distance);
  if (distance < 10){
  //movelikethis(CL, CR);
 digitalWrite(buzzer, HIGH);
  }
  else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2,LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4,LOW);
    }

//mpu6050



errorangle = desiredangle - checkangle();
CL = 150 + kp*errorangle;
CR = 150 - kp*errorangle;
CL = constrain(CL,-255,255);
CR = constrain(CR,-255,255);
movelikethis(CL, CR);
}
float checkangle(){
  mpu6050.update();
  float theta = mpu6050.getAngleZ();
  return theta;  
}
void movelikethis(int speedL, int speedR){
  analogWrite(ENB,abs(speedL));
  analogWrite(ENA,abs(speedR));
  if(speedL>0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2,LOW);
  }
  else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2,HIGH);
  }

  if(speedR>0){
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }
  else{
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }
}
