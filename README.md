# Final Project - Line Follower Robot

A Line Follower Robot (LFR) is one of the most important concepts in robotics. 
In this project we developed a platform that enables controlling the robot in 2 operating modes: A manual mode for robot maneuvering using joystick, and an automatic mode where the LFR comes into picture. The platform is controlled by the user with HTML-based GUI, where the LFR performance can be evaluated by several criteria.

Line identification is done using an array of five light reflecting sensors. Each sensor returns a value in the range [0-1000] that represents the amount of black it sees. It is common practice to use a formula where the sensors values are linearly weighted.

![image](https://user-images.githubusercontent.com/96314781/165382113-eb04646c-03de-488d-b368-f5290714e6a7.png)

In order to evaluate the line position more accurately, each sensor value can be proportionally weighted according to its real location. In this project, both methods were implemented.

The LFR consists of a Raspberry Pi and a Zumo 32U4 robot, connected by USB. The RPi runs an HTML-based GUI (Python) and communicates with the Zumo, that runs an Arduino sketch.

![image](https://user-images.githubusercontent.com/96314781/165382476-bd4f95f7-8f84-486c-bedd-a3c2f326244d.png)

![image](https://user-images.githubusercontent.com/96314781/165382549-6e67362b-b6bc-4c03-8180-78c1348d3159.png)

The PID controller produces speedDifference using the formula:
![image](https://user-images.githubusercontent.com/96314781/178268828-01d894d0-3389-4277-adbf-2c49f6d4e584.png)

Three parameters were defined to evaluate the performance:  
Average – average of line position samples.  
SD – standard deviation of the samples.  
Accuracy – percentage of samples within a predetermined range around the center.  
  
These parameters were inserted to the performance formula:

![image](https://user-images.githubusercontent.com/96314781/165382657-537e83aa-18e6-428d-a89c-e5482a121b5d.png)

The plots of the 3 performance evaluation parameters change in tune to the speed and the proportional gain.

![image](https://user-images.githubusercontent.com/96314781/165382839-d32fcb27-a898-460a-b02a-a2dd094bb425.png)

After some system optimizations, such as transitioning from a linear method of sensors distribution to a proportional one and increasing the sampling rate, the shape of the performance plot has dramatically changed.

![image](https://user-images.githubusercontent.com/96314781/165382954-cf26dee9-d286-42b6-87c1-ed137954514a.png)

In order to achieve better results, the step response was optimized using a PID controller. The overshoot and the rise time were reduced to the minimal values possible.

![image](https://user-images.githubusercontent.com/96314781/165383181-f3f8d490-8636-43dc-9579-edfe890f77ca.png)

**Video - Auto mode control:** Driving in auto mode, applying changes of speed and proportional gain (Kp) via sliders, examining effects on the line position plot and line sensors values bar chart.

https://user-images.githubusercontent.com/96314781/158193784-7cf57be2-ac55-4888-a2b4-42638e2c5b0a.mp4


**Video - basic operations:** The robot starts in manual mode and gets driven to the line. Then, the user switches to automatic mode which makes the robot calibrating and starting a ride following the black line on the ground. On the left there's a dynamic barchart showing the line sensors readings.

https://user-images.githubusercontent.com/96314781/150839280-9746bab1-2845-4fed-819e-54ee128ef291.mp4

**Video - Sweeping over the line sensors:** Presenting the dynamic line sensors barchart.

https://user-images.githubusercontent.com/96314781/160406284-dae29264-81fc-4346-9d95-b591768e68c4.mp4

## Lab Workspace

![Workspace overview](https://user-images.githubusercontent.com/96314781/150839941-7c10bb56-df51-4970-9f03-8f37131ef499.jpg)
