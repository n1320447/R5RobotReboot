#include <Wire.h>
#include <AS5600.h>
#include <TB6612_ESP32.h>
#include "BluetoothSerial.h"
#include "MotorExtended.h"
#include "DistanceSensor.h"


//I2C pins
#define SCL1 22
#define SCL2 32
#define SDA1 21
#define SDA2 33

//Motor driver pins
#define AIN1 13
#define BIN1 12
#define AIN2 14
#define BIN2 27
#define PWMA 26
#define PWMB 25
#define STBY 5

//Distance sensor pins
#define DIST1 34

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void bluetoothConnection(){
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);       
}


TwoWire I2Ctwo = TwoWire(1);

MotorExtended motor1(AIN1, AIN2, PWMA, STBY, 1, 1);
MotorExtended motor2(BIN1, BIN2, PWMB, STBY, 2, I2Ctwo, 1);
DistanceSensor dist1(DIST1);

void updateAll(MotorExtended m1, MotorExtended m2){
  Serial.println("Trying to update sensors");
  m1.updateEncoderPos();
  m2.updateEncoderPos();
}

void moveForward(int speed){
  motor1.analogDriveMotor(speed*1.00);
  motor2.analogDriveMotor(speed);
}

void moveReverse(){
  motor1.analogDriveMotor(50);
  motor2.analogDriveMotor(50);
  delay(1000); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void reverseFromLeft(){
  motor1.analogDriveMotor(100);
  motor2.analogDriveMotor(0);
  delay(1000); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void reverseFromRight(){
  motor1.analogDriveMotor(0);
  motor2.analogDriveMotor(100);
  delay(1000);
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void turnLeft(){

  motor1.analogDriveMotor(-100);
  motor2.analogDriveMotor(0);
  delay(800); // Wait for 1 second

  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void turnRight(){
  motor1.analogDriveMotor(0);
  motor2.analogDriveMotor(-100);
  delay(1000); // Wait for 1 second
  // Stop the motors after 1 second
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}

void scanLeftandRight(){
  //turn left a bit
  int found = 0;
  // motor1.analogDriveMotor(-50);
  // motor2.analogDriveMotor(0);
  // delay(2000);
  turnLeft();
  reverseFromLeft();
  turnRight();
  reverseFromRight();
  // turnRight();
  // motor1.analogDriveMotor(0);
  // motor2.analogDriveMotor(-50);
  // delay(3000);
   

}

void breakMotors(){
  motor1.brakeMotor(); // If there's a brake function
  motor2.brakeMotor(); // If there's a brake function
}


/* 
Function attempts to find clearance from sensor1, ideally
would use both sensors but sensor 2 is acting funky.
*/
void getClearOfObject(){

  // float distance1 = sensorValueToDistance(analogRead(DIST1));
  float distance1 = dist1.getDistance();
  //2.5" wheels currently
  // if (distance1 <= 20){
  int left = 0;
  int right = 0;
  // turnLeft();
    while(distance1 <= 10){
      // turnLeft();
      // scanLeftandRight();
      
      turnRight();
      distance1 = dist1.getDistance();
      if (distance1 <=10) right = 1;
      
      // if (distance1 <=20) right = 1;
      if (right == 0) 
      {
        Serial.print("this is right boolean: ");
        Serial.println();
        turnRight();
        break;
      }

      reverseFromRight();

      turnLeft();
      distance1 = dist1.getDistance();
      if(distance1 <= 10) left = 1;
      
      // if(distance1 <= 20) left = 1;
      if (left == 0) 
      {
        turnLeft();
        break;
      }

      reverseFromLeft();

      // if(left == 1 && right == 1){
      //   moveReverse();
      //   break;
      // } 
      
      if(left == 0 && right == 0){
        moveForward(-100);
        break;
      }
      // else if(left == 0 && right ==1){
      //   //go left, its clear
      //   turnLeft();
      //   delay(100);
      //   moveForward(-100);
      // } else if(left == 1 && right == 0){
      //   turnRight();
      //   delay(100);

      //   moveForward(-100);
      // }
      distance1 = dist1.getDistance();
      // left = 0;
      // right = 0;
    }
}

//Seed round code
void seedRound(){

    // turn right from starting postion then turn left. 
    // Gets ready to start moving forward towards other end of course
    turnRight();
    moveForward(-150);
    delay(250);
    turnLeft();
    delay(250);
    Serial.println("Start moving forward...");


    // Loops that makes robot move forward if object detected, 
    // runs getClearOfObject().
  
    // while(!objectFound(sensorValueToDistance(analogRead(DIST1)))){
    int objectsDetected = 0; // int saving the amount of objects detected.
    while(true){
      // while(true){
      Serial.println(dist1.getDistance());
      moveForward(-155);
      if(dist1.objectFound()){
        Serial.println("found object dist1");
        getClearOfObject();
        objectsDetected++;
        Serial.print("objectDetect:");
        // Serial.println(objectsDetected);
      } 
    }

    Serial.println("found object dist1!!!!");
    
    Serial.println("moving left...");
    moveReverse();
    //turn to start navigating back, need to get to other side of T-posts
    turnLeft();
    moveForward(-150);
    delay(1000);
    //turn towards other side of t-posts
    Serial.println("turning right...");
    turnRight();
    moveForward(-150);

  delay(1000);
  Serial.println("break motors...");
  breakMotors();

}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Beginning connect process...");
  I2Ctwo.begin(SDA2, SCL2, 100000);
  delay(1000);

  Serial.println("For motor1...");
  motor1.connectEncoder();
  Serial.println("For motor2...");
  motor2.connectEncoder();


  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  bluetoothConnection();
  seedRound();
}

void loop() {
}