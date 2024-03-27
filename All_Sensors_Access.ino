#include <Wire.h>
#include <AS5600.h>
#include <TB6612_ESP32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include "BluetoothSerial.h"


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
#define DIST2 4 //DOES NOT WORKKKKKKKKKKK, will not work, no ADC pins left on esp :( without pretty significant changes to wiring will need to make due with these
#define DIST3 2

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

const char* ssid = "";     // Replace with your Wi-Fi network name
const char* password = ""; // Replace with your Wi-Fi password



class MotorExtended {
  public:
      //Encoder values
      long cumulative_position;//between 0-4096 represents angle
      long previous_cumulative_position;

      float angular_velocity;//in mm/s
      float previous_angular_velocity;

      int analog_velocity; //btwn -255, 255 (for directly controlling motor)
      int prev_analog_velocity;

      long offset_cumulative_position;//in 0-4096 represents angle
      float radians_per_tick = 0.0015343553863686413; //2pi/4096

      //General Values
      long prev_read_time = millis();//in millis
      float linear_velocity; //in mm

      //Physical params
      const float diameter = 51.00;//in mm

      // Member variables for motor and encoder
      Motor motor;
      AS5600 encoder;

    // Constructor for non-default I2C bus
    MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, TwoWire& wire, int DIRECTION = 1) //change to -1 to change direction
      : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), 
        encoder(&wire){
    }
    // Constructor for using the default I2C bus
    MotorExtended(int IN1, int IN2, int PWM, int STBY_PIN, int CHANNEL, int DIRECTION = 1)
    : motor(IN1, IN2, PWM, DIRECTION, STBY_PIN, 5000, 8, CHANNEL), encoder(&Wire) {
        // Initialization using the default I2C bus
    }


    void connectEncoder(){
      //Try to connect to motor
      while (!encoder.isConnected()) {
          Serial.println("Trying to connect...");
          encoder.begin();
          delay(1000);
      }

      Serial.println("Connected!");

      // Find and set offset for encoders
      offset_cumulative_position = encoder.getCumulativePosition();
    }

    void updateEncoderPos(){
      /*
      Updates the encoder position attribute
      */

      //updates previous position
      if(previous_cumulative_position){
        previous_cumulative_position = cumulative_position;
      } else{
        previous_cumulative_position = 0;
      }

      //updates time
      prev_read_time = millis();

      //gets new position
      cumulative_position = encoder.getCumulativePosition() - offset_cumulative_position;
    }




    void updateAngularVelocity(bool smoothing = true, float alpha = 0.1){
      // Ensure there is a new position update
      long current_read_time = millis();
      long delta_t = current_read_time - prev_read_time;

      if (delta_t > 0) { // Check to prevent division by zero
        updateEncoderPos(); // Updates cumulative_position

        float delta_t_seconds = float(delta_t) / 1000.0;
        float delta_pos = (cumulative_position - previous_cumulative_position) * radians_per_tick;
        float temp_angular_velocity = delta_pos / delta_t_seconds; // radians per second

        // Perform smoothing if desired
        if (!smoothing) {
          angular_velocity = temp_angular_velocity;
        } else {
          angular_velocity = (previous_angular_velocity * alpha) + (temp_angular_velocity * (1 - alpha));
        }

        // Update previous values for the next calculation
        previous_cumulative_position = cumulative_position;
        previous_angular_velocity = angular_velocity;
        prev_read_time = current_read_time;
      }
    }

    void analogDriveMotor(int analogSpeed){
      /*
      Does basic drive for motor by providing value between -255, 255
      TODO: implement an amount of time to drive for without blocking
      */
      prev_analog_velocity = analog_velocity;
      analog_velocity = analogSpeed;

      motor.drive(analog_velocity);
    }

    void brakeMotor(){
      /*
      Brakes the motor
      */
      motor.brake();
    }

    void velocityDriveMotor(float desiredAngularVelocity, float thresholdVelocityDifference = 10.0, int analogStep = 5){
      /*
      Tries to bring motor up to a desired radian/second velocity
      Note:
        -needs to have some threshold difference or loop will never exit, still need to tune
        -can adjust analogStep, higher value is faster acceleration/loop
      TODO:
        -Replace with PID controller, it is the better way to do this
      */
      float velocity_diff = 0.0;
      bool exit_loop = false;

      while(!exit_loop){
        updateAngularVelocity();
        velocity_diff = desiredAngularVelocity - angular_velocity;
        if(abs(velocity_diff) >= thresholdVelocityDifference){//If you are not already at the speed...
          if(velocity_diff < 0){//if you need to accelerate...
            if(analog_velocity <= (255 - analogStep)){//and if the motor can still go faster
              analogDriveMotor((analog_velocity + analogStep)); //step the motor speed up a notch
            } else{
              exit_loop = true;
            }
          }else if (velocity_diff > 0){//if you need to decelerate...
            if(analog_velocity >= (-255 + analogStep)){//and the motor can still slow more...
              analogDriveMotor((analog_velocity - analogStep)); //step the motor speed down an notch
            } else {
              exit_loop = true;
            }
          }
        } else {
          exit_loop = true;
        }
      }
    }
};

class DistanceSensor {
  public:
    int pin;

    int analog_distance;
    float current_distance;
    float previous_distance;


    DistanceSensor(int DATA_PIN){
      pin = DATA_PIN;
      pinMode(DATA_PIN, INPUT);
    }

    float mapDataToDist(int analog_value){
      //Dummy Function for now
      previous_distance = current_distance;
      float dist = float(analog_value);
      current_distance = dist;
      return dist;
    }

    void readSensor(){
      //Get the raw distance from sensor
      int raw_data = analogRead(pin);
      analog_distance = raw_data;
    }

    float getDistance(){
      //Updates, maps, then returns end distance from sensor
      readSensor();
      return analog_distance;
      // return(mapDataToDist(analog_distance));
    }
};

class IMU {
  public:
    long prev_read_time = 0;

    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    float ang_vel_x = 0; // Gyro angular velocity X
    float ang_vel_y = 0; // Gyro angular velocity Y
    float ang_vel_z = 0; // Gyro angular velocity Z

    // Filtered angles
    float angle_x = 0; // Roll
    float angle_y = 0; // Pitch

    Adafruit_MPU6050 imu;

    IMU() {}

    bool begin() {
        if (!imu.begin()) {
            Serial.println("Failed to find MPU6050 chip");
            return false;
        }
        imu.setAccelerometerRange(MPU6050_RANGE_8_G);
        imu.setGyroRange(MPU6050_RANGE_500_DEG);
        imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("MPU6050 Found!");
        prev_read_time = millis();
        return true;
    }

    void connect(int timeout = 10000){
      long start_time = millis();
      long time_spent = 0;
      while(!begin() && time_spent < timeout){
        Serial.println("IMU not connected...");
        time_spent = millis() - start_time;
        delay(100);
      }
      if(!begin()){
        Serial.println("Could not connect");
      } else{
        Serial.println("Connected to IMU!");
      }

    }

    void update() {
        sensors_event_t a, g, temp;
        imu.getEvent(&a, &g, &temp);

        // Current time in milliseconds
        long current_time = millis();
        float dt = (current_time - prev_read_time) / 1000.0f; // Delta time in seconds
        prev_read_time = current_time;

        // Update acceleration (m/s^2)
        accel_x = a.acceleration.x;
        accel_y = a.acceleration.y;
        accel_z = a.acceleration.z;

        // Update angular velocities (rad/s)
        ang_vel_x = g.gyro.x * (PI / 180.0);
        ang_vel_y = g.gyro.y * (PI / 180.0);
        ang_vel_z = g.gyro.z * (PI / 180.0);

        // Calculate accelerometer angles
        float accel_angle_x = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * (180.0 / PI);
        float accel_angle_y = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * (180.0 / PI);

        // Complementary filter
        // Combine accelerometer and gyroscope readings for roll and pitch angles
        const float alpha = 0.98;
        angle_x = alpha * (angle_x + ang_vel_x * dt) + (1.0 - alpha) * accel_angle_x;
        angle_y = alpha * (angle_y + ang_vel_y * dt) + (1.0 - alpha) * accel_angle_y;
    }

    void printValues() {
        Serial.print("Roll (X): "); Serial.print(angle_x); Serial.println(" deg");
        Serial.print("Pitch (Y): "); Serial.print(angle_y); Serial.println(" deg");
    }
};

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
IMU imu;
DistanceSensor dist1(DIST1);
DistanceSensor dist2(DIST3);

void updateAll(MotorExtended m1, MotorExtended m2){
  Serial.println("Trying to update sensors");
  m1.updateAngularVelocity();
  m2.updateAngularVelocity();
  m1.updateEncoderPos();
  m2.updateEncoderPos();
}

/// SENSOR PROPERTIES
float distance_prev = 0.0;
float prev_time = 0.0;
const size_t ROLLING_AVERAGE_SIZE = 20;
const float SPIKE_MASK = 1.3;
float rolling_average[ROLLING_AVERAGE_SIZE] = {};
size_t arr_i = 0;

/// TURN SENSOR VALUE TO DISTANCE VALUE
float sensorValueToDistance(int value){
  float distance_cm = 28000.0/value;

  float cur_time = micros() / 1000000.0;
  float slope = (distance_cm - distance_prev) / (cur_time - prev_time);

  if (distance_cm == INFINITY) {
    return 0.0;
  }
  rolling_average[arr_i] = distance_cm;
  arr_i = (arr_i + 1) % ROLLING_AVERAGE_SIZE;

  float sum = 0.0;
  for (size_t i = 0; i < ROLLING_AVERAGE_SIZE; ++i)
    sum += rolling_average[i];
  // Serial.println(sum / ROLLING_AVERAGE_SIZE, 2);

  distance_prev = distance_cm;
  prev_time = cur_time;

  if (value == 0) {
    // Handle invalid reading
    return -1;
  }
  float voltage = value * (3.3 / 4095.0);
  if (voltage == 0) {
    // Handle invalid voltage
    return -1;
  }
  
  return distance_cm;
}

/*
psuedoCode for seeding round
Code should make robot navigate from one end of the course
to the other, There may or may not be any obstacle on the
way to the other end.

Best Case: 
  1)navigate straight from the starting point,
  2)IR sensor senses end of course, adjust robot to face button
  3) push button
  4) turn robot countclockwise
    4a) This will need to be done based on clearances
        of sensors readings. maybe turn a few degrees, check readings,
        turn a few more degrees,
    4b) once we have enouch clearance start moving bot to other end
  5) Keep moving robot until end of course is met.

Worst Case:
  7) Same as above but will have obstacles during traveling
  from 1 end to another.

Match:
- can use a few functions to trigger when an object is found

Plan:
  functions:
  ObjectFound() -> Bool : trigger some code if object is found
  forward() -> make motors move in straight line
  reverse() -> make robot move backwards in a straight line
  turnLeft() -> Void: turns robot to left
  turnRight() -> Void: turns robot right
  getClearOfObstacle() -> Bool: checks what sensors have an object close,
  turn left or right to get clear of object.

  
*/

bool objectFound(int value){
  //object detected less <= to 20 cm
  if (value <= 20){
    return true;
  }
  return false;
}


/*new code starts here
  input: int speed
  desription: parameter speed will dictate how fast motors go.
  a neg value goes forward, pos goes backward
  output: return void
*/ 

void moveForward(int speed){
  motor1.analogDriveMotor(speed*1.04);
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
  delay(1200); // Wait for 1 second

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

  float distance1 = sensorValueToDistance(analogRead(DIST1));
  //2.5" wheels currently
  if (distance1 <= 20){
    int left = 0;
    int right = 0;
    // turnLeft();
      while(distance1 <= 20){
        // turnLeft();
        // scanLeftandRight();
        turnRight();
        if (distance1 <=20) left = 1;
        reverseFromRight();
        if (distance1 <=20) left = 1;
        
        turnLeft();
        if(distance1 <= 20) right = 1;
        reverseFromLeft();
        if(distance1 <= 20) right = 1;

        if(left == 1 && right == 1){
          moveReverse();
          
        } else if(left == 0 && right ==1){
          //go left, its clear
          turnLeft();
          moveForward(-100);
        } else if(left == 1 && right == 0){
          turnRight();
          moveForward(-100);
        }
        distance1 = sensorValueToDistance(analogRead(DIST1));
      }
      
  }

  // float distance3 = sensorValueToDistance(analogRead(DIST3));
  // if (distance3 <= 15){
  //   while(distance3 <= 15){
  //     turnLeft();
  //     distance3 = sensorValueToDistance(analogRead(DIST3));
  //     if(distance3>15) break;
  //   }
  // }

}

//Seed round code
void seedRound(){
    // motor1.analogDriveMotor(-100);
    // motor2.analogDriveMotor(-105);


    // turn right from starting postion
    turnRight();
    moveForward(-100);
    delay(250);
    turnLeft();
    //move forward
    Serial.println("moving forward...");
    moveForward(-100);
    delay(10000); // need to fine tune this, this 
    //approach side wall, turn left
    Serial.println("moving left...");
    // turnLeft();
    // delay(1000); // need to fine tune this
    // Serial.println("moving forward...");
    // moveForward(-100);
    
    //try to hit middle button? need to add midpoint check for mid button

    //approach other end of seeding round course
    if(objectFound(sensorValueToDistance(analogRead(DIST1)))){
      Serial.println("found object dist1");
      getClearOfObject();
      //turn towards end button
      turnLeft();
      moveForward(-100);
      //push button
      //reverse from buttton
    } 

    moveReverse();
    //turn to start navigating back, need to get to other side of T-posts
    turnLeft();
    moveForward(-100);
    delay(1000);
    //turn towards other side of t-posts
    Serial.println("turning right...");
    turnRight();
    moveForward(-100);

  delay(1000);
  Serial.println("break motors...");
  breakMotors();

}

void eliminationRound(){

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
  Serial.println("For imu...");
  imu.connect();
  //  Serial.begin(115200);

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  bluetoothConnection();
  seedRound();

  
}

void loop() {

  // Serial.println(sensorValueToDistance(analogRead(DIST1)));
  // bluetoothConnection();
  // seedRound();
}

