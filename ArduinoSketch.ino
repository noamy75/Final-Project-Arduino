#include <ArduinoJson.h> // IDE
#include <Wire.h>       //IDE
#include <Zumo32U4.h>    // IDE

// Definitions and declarations

#define NUM_LINE_SENSORS 5
#define NUM_PROX_SENSORS 4
#define NUM_IMU_SENSORS 2
#define MAX_NUM_SENSORS 5
#define ONE_CYCLE_DISTANCE 12.17367 // (38.75*Pi/10) - The distance in centimeters of one motor cycle (1200 encoder pulses)
#define ENCODER_PULSES_PER_CYCLE 1200
 
Zumo32U4ProximitySensors prox;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

String inputString;
unsigned int lineSensorValues[NUM_LINE_SENSORS];
unsigned int proxSensorValues[NUM_PROX_SENSORS];
int16_t imuSensorValues[NUM_IMU_SENSORS];
float joyX;
float joyY;
bool calibration_flag = true;
const uint16_t maxSpeed = 200;
const uint16_t baseSpeed = 100;
int16_t lastError = 0;
int rightMotor = 0;
int leftMotor = 0;
bool stringComplete;
int16_t position = 0;
bool fork_flag = 0;
float Kp;
float Kd;
float Ki;
int16_t error_sum = 0;
int Speed;
unsigned long last_timestamp = 0; // timestamp for dT for control loop
unsigned long current_timestamp = 0; // timestamp for dT for control loop
int DT = 0; // dT for control loop
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

//functions

// Operates the same as read calibrated, but also returns an
// estimated position of the robot with respect to a line. The
// estimate is made using a weighted average of the sensor indices
// multiplied by 1000, so that a return value of 0 indicates that
// the line is directly below sensor 0, a return value of 1000
// indicates that the line is directly below sensor 1, 2000
// indicates that it's below sensor 2000, etc.  Intermediate
// values indicate that the line is between two sensors.  The
// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
//
// By default, this function assumes a dark line (high values)
// surrounded by white (low values).  If your line is light on
// black, set the optional second argument white_line to true.  In
// this case, each sensor value will be replaced by (1000-value)
// before the averaging.
int getLinePosition(unsigned int *sensor_values,
    unsigned int direction, unsigned int position, bool calibration_flag)
{
    unsigned char i, on_line = 0;
    unsigned long avg, avg1, avg2; // this is for the weighted total, which is long before division
    unsigned int sum, sum1, sum2; // this is for the denominator which is <= 64000
    avg = 0;
    avg1 = 0;
    avg2 = 0;
    sum = 0;
    sum1 = 0;
    sum2 = 0;

    for(i=0;i<NUM_LINE_SENSORS;i++) {
        int value = sensor_values[i];

        // keep track of whether we see the line at all
        if(value > 400) {
            on_line = 1;
        }

        // only average in values that are above a noise threshold
        if(value > 50) {
            
            // Linear distribution of line sensors
            /*
            avg += (long)(value) * (i * 1000); 
            */
            
            // Proportional distribution of line sensors
            //
            if(i == 1){
              avg += (long)(value) * 1600;
            }
            if(i == 3){
              avg += (long)(value) * 2400;
            }
            if(i == 0 || i == 2 || i == 4){
              avg += (long)(value) * (i * 1000);
            }
            //
            
            sum += value;
            if (i < 2)
            {
              avg1 += (long)(value) * (i * 1000);
              sum1 += value;
            }
            if (i > 2)
            {
              avg2 += (long)(value) * (i * 1000);
              sum2 += value;
            }
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
        if(position < (NUM_LINE_SENSORS-1)*1000/2)
            return 0;

        // If it last read to the right of center, return the max.
        else
            return (NUM_LINE_SENSORS-1)*1000;

    }

    // Fork in the road
    if ((sum1 > 400 && sum2 > 400))
    {
      fork_flag = 1;
      //if (position < (NUM_LINE_SENSORS-1)*1000/2)
      if (direction == 0)
      {
        position = avg1/sum1;
      }
      else
      {
        position = avg2/sum2;
      }     
    }
    else
    {
      fork_flag = 0;
      position = avg/sum;
    }
    return position;
}

void calibrateSensors()
{

  // Wait 0.2 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(200);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void getImuSensorValues() {
  imu.read();
  imuSensorValues[0] = imu.a.x;
  imuSensorValues[1] = imu.a.y;
}

void getLineSensorValues(){
  lineSensors.readCalibrated(lineSensorValues);
}

void getProxSensorValues(){
  prox.read();
  proxSensorValues[0] = prox.countsLeftWithLeftLeds();
  proxSensorValues[1] = prox.countsFrontWithLeftLeds();
  proxSensorValues[2] = prox.countsFrontWithRightLeds();
  proxSensorValues[3] = prox.countsRightWithRightLeds();
}

// main functions

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  imu.init();
  imu.enableDefault();
  prox.initThreeSensors();
  delay(1000);
  lineSensors.initFiveSensors();
  delay(1000);
  Serial.begin(9600);
  inputString.reserve(200);
}

void loop() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  // print the string when a newline arrives:
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
    int joyX = str.toInt();
    
    int ind4 = inputString.indexOf('?');  // find location of fourth delimiter '?'
    str = inputString.substring(ind3 + 1, ind4);   // capture fourth data String - joyY
    int joyY = str.toInt();
    
    int ind5 = inputString.indexOf('!');  // find location of fifth delimiter '!'
    str = inputString.substring(ind4 + 1, ind5);   // capture fifth data String - Kp
    float Kp = str.toFloat();

    int ind6 = inputString.indexOf('@');  // find location of sixth delimiter '@'
    str = inputString.substring(ind5 + 1, ind6);   // capture sixth data String - Kd
    float Kd = str.toFloat();

    int ind7 = inputString.indexOf('#');  // find location of seventh delimiter '#'
    str = inputString.substring(ind6 + 1, ind7);   // capture seventh data String - Ki
    float Ki = str.toFloat();

    str = inputString.substring(ind7 + 1);   // capture eighth data String - Kd
    int baseSpeed = str.toInt();
    int maxSpeed = baseSpeed + 100;

    // Automatic
    if(auto_mode == 1){
        if(calibration_flag){
          calibrateSensors();
          error_sum = 0;
        }
      getLineSensorValues();
      position = getLinePosition(lineSensorValues, direction, position, calibration_flag);
      int16_t error = position - 2000;
      int16_t speedDifference = 0;
      error_sum += error;
      constrain(error_sum, -5000, 5000);
      last_timestamp = current_timestamp;
      current_timestamp = millis();
      DT = current_timestamp - last_timestamp;
      if (fork_flag == 0)
      {
        speedDifference = error * Kp + (Kd/DT) * (error - lastError) + (Ki*DT) * error_sum;
      }
      else
      {
        speedDifference = error + 20 * (error - lastError);
      }
      lastError = error;
      int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
      int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;
      leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
      rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
      motors.setSpeeds(leftSpeed, rightSpeed);
      calibration_flag = false;
    }

    // Manual
    else{
      leftMotor = joyY + int(float(joyX)/1.5);
      rightMotor = joyY - int(float(joyX)/1.5);
      motors.setLeftSpeed(leftMotor);
      motors.setRightSpeed(rightMotor);
      calibration_flag = true;
    }

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
    getImuSensorValues();
    getLineSensorValues();
    getProxSensorValues();

    // Read battery level
    uint16_t batteryLevel = readBatteryMillivolts();
    float battery = float(batteryLevel)/1000.0;

    // Send data to Pi by json document
    // data includes timestamp, battery and sensor values which are stored in arrays
    DynamicJsonDocument jBuffer(1024);
    JsonArray line_sensor_values = jBuffer.createNestedArray("line_sensor_values");
    JsonArray imu_sensor_values = jBuffer.createNestedArray("imu_sensor_values");
    JsonArray prox_sensor_values = jBuffer.createNestedArray("prox_sensor_values");
    jBuffer["DT"] = DT;
    jBuffer["timestamp"] = current_timestamp;
    jBuffer["Battery"] = battery;
    jBuffer["Position"] = position;
    jBuffer["Velocity"] = total_velocity;
    jBuffer["left_velocity"] = left_velocity;
    jBuffer["right_velocity"] = right_velocity;
    // fill each key's values according to the sensors values
    for (int i = 0; i < MAX_NUM_SENSORS; i++){
      if( i < NUM_IMU_SENSORS){
        imu_sensor_values.add(imuSensorValues[i]);
      }
      if (i < NUM_LINE_SENSORS){
        line_sensor_values.add(lineSensorValues[i]);
      }
      if(i < NUM_PROX_SENSORS){
      prox_sensor_values.add(proxSensorValues[i]);
      }
    }

    // document transmission
    serializeJson(jBuffer, Serial);
    Serial.println();
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
