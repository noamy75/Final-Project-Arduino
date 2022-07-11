#include <ArduinoJson.h> // IDE
#include <Wire.h>        //IDE
#include <Zumo32U4.h>    // IDE

/******************************
Definitions and Declarations
*******************************/

#define NUM_LINE_SENSORS 5
#define MAX_NUM_SENSORS 5
#define ONE_CYCLE_DISTANCE 12.17367 // (38.75*Pi/10) - The distance in cm of one wheel cycle. diameter = 38.75mm.
#define ENCODER_PULSES_PER_CYCLE 1200 // There're 1200 encoder pulses in a single wheel cycle
#define LINE_THRESHOLD 400
 
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

String inputString;
bool stringComplete;
unsigned int lineSensorValues[NUM_LINE_SENSORS];
float joyX, joyY;
bool calibration_flag = true;
uint16_t maxSpeed = 200;
uint16_t baseSpeed = 100;
int16_t lastError = 0;
int rightMotor = 0;
int leftMotor = 0;
int16_t position = 0;
bool fork_flag = 0;
float Kp, Kd, Ki;
int Speed;
int16_t error_sum = 0; // errors sum for control loop - integral term 
unsigned long last_timestamp = 0; // timestamp for dT for control loop - differential term
unsigned long current_timestamp = 0; // timestamp for dT for control loop - differential term
int DT = 0; // dT for control loop - differential term
int total_DT = 0; // total time from start for control loop - integral term
unsigned long velocity_t1 = 0; // first timestamp for dT for velocity calculation
unsigned long velocity_t2 = 0; // second timestamp for dT for velocity calculation
float velocity_DT = 0; // dT for velocity calculation 
float left_distance = 0;
float right_distance = 0;
float left_velocity = 0;
float right_velocity = 0;
float total_velocity = 0;
long left_counts = 0;
long right_counts = 0;

/******************************
Functions
*******************************/

/* This function returns an estimated position of the robot with respect to a line. 
 * The estimate is made using a weighted average of the sensor indices multiplied
 * by 1000, so that a return value of 0 indicates that the line is directly below
 * sensor 0, a return value of 1000 indicates that the line is directly below
 * sensor 1, 2000 indicates that it's below sensor 2000, etc. Intermediate values
 * indicate that the line is between two sensors. The formula is:

    0*value0 + 1000*value1 + 2000*value2 + ...
   --------------------------------------------
         value0  +  value1  +  value2 + ...
*/
int getLinePosition(unsigned int *sensor_values,
    unsigned int direction, unsigned int position, bool calibration_flag)
{
    unsigned char i, on_line = 0;
    unsigned long avg, avg1, avg2; // this is for the weighted total, which is long before division
    unsigned int sum, sum1, sum2; // this is for the denominator which is <= 64000
    avg = 0; avg1 = 0; avg2 = 0;
    sum = 0; sum1 = 0; sum2 = 0;

    for(i = 0; i < NUM_LINE_SENSORS; i++) {
        int value = sensor_values[i];

        // keep track of whether we see the line at all
        if(value > LINE_THRESHOLD) 
            on_line = 1;

        // only average in values that are above a noise threshold
        if(value > 50) {
            
            // Linear distribution of line sensors
            /*
            avg += (long)(value) * (i * 1000); 
            */
            
            // Proportional distribution of line sensors
            //
            if(i == 1)
              avg += (long)(value) * 1600;
            
            if(i == 3)
              avg += (long)(value) * 2400;
            
            if(i == 0 || i == 2 || i == 4)
              avg += (long)(value) * (i * 1000);
            //
            
            sum += value;
            if (i < 2){
              avg1 += (long)(value) * (i * 1000);
              sum1 += value;
            }
            
            if (i > 2){
              avg2 += (long)(value) * (i * 1000);
              sum2 += value;
            }
        }
    }

    // decide position in case no sensor sees the line
    if(!on_line){
        // if the last read was to the left of center, return 0.
        if(position < (NUM_LINE_SENSORS-1)*1000/2)
            return 0;

        // if the last read was to the right of center, return max.
        else
            return (NUM_LINE_SENSORS-1)*1000;
    }

    // fork is in the road, decide which path to take
    if (sum1 > LINE_THRESHOLD && sum2 > LINE_THRESHOLD){
      fork_flag = 1;

      // the robot's driving direction is counter-clockwise
      if (direction == 0)
        // position is based only on left sensors
        position = avg1/sum1;
      
      // the robot's driving direction is clockwise
      else
        // position is based only on right sensors
        position = avg2/sum2;  
    }
    
    // no junction, regular position calculation
    else{
      fork_flag = 0;
      position = avg/sum;
    }
    
    return position;
}

/*  This function calibrates the line sensors by sweeping them
 *  across the line and mesuring the minimal and maximal light 
 *  values it detects.
 */
void calibrateSensors(){
  // Wait 200 milliseconds and then begin automatic sensor calibration
  // by rotating in place and sweeping the line sensors across the line.
  delay(200);
  for(uint16_t i = 0; i < 120; i++){
    if (i > 30 && i <= 90){
      motors.setSpeeds(-200, 200);
    }
    else{
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

/* This function fills the array lineSensorValues with
 * the calibrated values.
 */
void getLineSensorValues(){
  lineSensors.readCalibrated(lineSensorValues);
}

/******************************
Setup and Loop
*******************************/

void setup() {
  // initialize all five line sensors
  lineSensors.initFiveSensors();
  delay(1000);
  
  // exchange messages with the Serial Monitor at a data rate of 9600 bits per second
  Serial.begin(9600);
  
  // allocate a buffer of 200 bytes in memory
  inputString.reserve(200);
}

void loop() {
  while (Serial.available()) {
    // get the new byte
    char inChar = (char)Serial.read();
    // add it to the inputString
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  // Parse message
  if (stringComplete) {
    String str;
    
    int ind1 = inputString.indexOf(';');  // find location of first delimiter ';'
    str = inputString.substring(0, ind1);   // capture first data String - auto_mode flag 
    int auto_mode = str.toInt();
    
    int ind2 = inputString.indexOf('.'); // find location of second delimiter '.'
    str = inputString.substring(ind1 + 1, ind2);   // capture second data String - direction
    int direction = str.toInt(); 
    
    int ind3 = inputString.indexOf(',');  // find location of third delimiter ','
    str = inputString.substring(ind2 + 1, ind3);   // capture third data String - joyX
    joyX = str.toInt();
    
    int ind4 = inputString.indexOf('?');  // find location of fourth delimiter '?'
    str = inputString.substring(ind3 + 1, ind4);   // capture fourth data String - joyY
    joyY = str.toInt();
    
    int ind5 = inputString.indexOf('!');  // find location of fifth delimiter '!'
    str = inputString.substring(ind4 + 1, ind5);   // capture fifth data String - Kp
    float Kp = str.toFloat();

    int ind6 = inputString.indexOf('@');  // find location of sixth delimiter '@'
    str = inputString.substring(ind5 + 1, ind6);   // capture sixth data String - Kd
    Kd = str.toFloat();

    int ind7 = inputString.indexOf('#');  // find location of seventh delimiter '#'
    str = inputString.substring(ind6 + 1, ind7);   // capture seventh data String - Ki
    Ki = str.toFloat();

    str = inputString.substring(ind7 + 1);   // capture eighth data String - baseSpeed
    baseSpeed = str.toInt();
    maxSpeed = baseSpeed + 100;

    // Robot is in Automatic mode
    if(auto_mode == 1){
        // calibration occurs once when moving from manual mode to auto mode
        if(calibration_flag){
          calibrateSensors();
          error_sum = 0;
          total_DT = 0;
        }

      // get line sensors values and calculate position
      getLineSensorValues();
      position = getLinePosition(lineSensorValues, direction, position, calibration_flag);
      
      // calculate error for differential term and error_sum for integral term
      int16_t error = position - 2000;
      error_sum += error;
      constrain(error_sum, -5000, 5000); // truncation
      
      // calculate DT for differential term and total_DT for integral term
      last_timestamp = current_timestamp;
      current_timestamp = millis();
      DT = current_timestamp - last_timestamp;
      total_DT += DT;

      // PID controller calculates speedDifference
      int16_t speedDifference = 0;
      if (fork_flag == 0){
        speedDifference = Kp * error + (Kd/DT) * (error - lastError) + (Ki*total_DT) * error_sum;
      }

      // in case of a junction take a sharp turn
      else{
        speedDifference = error + 20 * (error - lastError);
      }
      
      // calculate new speeds
      int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
      int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;
      
      // truncate speeds if they exceed maxSpeed
      leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
      rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
      
      // apply new speeds
      motors.setSpeeds(leftSpeed, rightSpeed);

      // update calibration_flag and lastError
      lastError = error;
      calibration_flag = false;
    }

    // Robot is in Manual mode
    else{
      // calculate left motor and right motor speeds
      leftMotor = joyY + int(float(joyX)/1.5);
      rightMotor = joyY - int(float(joyX)/1.5);

      // apply new speeds
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(rightMotor);

      // update calibration_flag
      calibration_flag = true;
    }

    /* Calculating the robot's velocity from its encoders. There're 1200 encoder
     * counts in a single wheel turn. From the number of encoder counts occured 
     * since the last measurement we can calculate the distance.
     */
    left_counts = encoders.getCountsAndResetLeft(); // left encoder counts
    right_counts = encoders.getCountsAndResetRight(); // right encoder counts
    left_distance = (float(left_counts)/ENCODER_PULSES_PER_CYCLE)*ONE_CYCLE_DISTANCE; // calculate left distance using relative part of ONE_CYCLE_DISTANCE
    right_distance = (float(right_counts)/ENCODER_PULSES_PER_CYCLE)*ONE_CYCLE_DISTANCE; // calculate right distance using relative part of ONE_CYCLE_DISTANCE
    velocity_t1 = velocity_t2;
    velocity_t2 = millis();
    velocity_DT = 0.001*float((velocity_t2 - velocity_t1)); // DT in seconds
    left_velocity = left_distance / velocity_DT; // calculate left velocity
    right_velocity = right_distance / velocity_DT; // calculate right velocity
    total_velocity = 0.5*(left_velocity+right_velocity); // total velocity is the average of left and right velocities

    // Assign sensor values into sensor arrays
    getLineSensorValues();

    // Read battery level
    uint16_t batteryLevel = readBatteryMillivolts();
    float battery = float(batteryLevel)/1000.0; // convert millivolts to volts

    // Send data to the RPi via Json document
    DynamicJsonDocument jBuffer(1024);
    JsonArray line_sensor_values = jBuffer.createNestedArray("line_sensor_values");
    jBuffer["DT"] = DT;
    jBuffer["timestamp"] = current_timestamp;
    jBuffer["Battery"] = battery;
    jBuffer["Position"] = position;
    jBuffer["Velocity"] = total_velocity;
    jBuffer["left_velocity"] = left_velocity;
    jBuffer["right_velocity"] = right_velocity;
    
    // fill each key's values according to the sensors values
    for (int i = 0; i < NUM_LINE_SENSORS; i++){
        line_sensor_values.add(lineSensorValues[i]);
    }

    // document transmission
    serializeJson(jBuffer, Serial);
    Serial.println();
    
    // clear the string
    inputString = "";
    stringComplete = false;
  }
}
