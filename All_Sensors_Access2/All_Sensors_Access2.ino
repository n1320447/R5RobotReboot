#include <Wire.h>
#include <AS5600.h>
#include "BluetoothSerial.h"
#include "MotorExtended.h"
#include "DistanceSensor.h"
#include "Robot.h" // Include the Robot class header file

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

// Create a Robot object with motor and distance sensor configurations
Robot myRobot(AIN1, AIN2, PWMA, STBY, BIN1, BIN2, PWMB, DIST1, I2Ctwo);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  Serial.println("Beginning connect process...");
  I2Ctwo.begin(SDA2, SCL2, 100000);
  delay(1000);

  Serial.println("For motor1...");
  myRobot.motor1.connectEncoder(); // Access motor1 through myRobot object
  Serial.println("For motor2...");
  myRobot.motor2.connectEncoder(); // Access motor2 through myRobot object

  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  bluetoothConnection();
  // myRobot.seedRound(); // Call the seedRound function using the myRobot object
  myRobot.driveInSquare();

  Serial.println("first print");
  myRobot.occupancyGrid.printGrid();
  Serial.println("mark grid");
  myRobot.occupancyGrid.markCell(7, 7, OCCUPIED); // marks robot starting position square is 4"x4"
  Serial.println(" print again");
  myRobot.occupancyGrid.printGrid();
}

void loop() {
}