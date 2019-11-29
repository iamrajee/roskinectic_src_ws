<----------- Read me ---------->
Folder name : Optical encoder

Files attached in the folder:
1. Arduino code.ino
2. Timerone.cpp
3. Timerone.h
4. Results.doc

AIM Of the Project

To find the location of the vehicle at certain instant of time and
Map the path followed by the vehicle

COMPONENTS Required

1.Vehicle Chassis
2.Arduino UNO
3.L293D Motor Driver
4.DC Motors
5.Optical encoders
6.Breadboard
7.Jumper wires

WORKING

#The motor driver is connected with the DC motors to control the speed
#The optical encoders read the rotation data of the wheel
#The speed of rotation of each wheel is used to find its translational 
 velocity and the direction of the body 
#The abcissa andordinate of the vehicle is updated at every second from
 reference (0 , 0)

RESULT

The obtained results were plotted with help of MATLAB

REFERENCE

The Timerone.h and Timerone.cpp were taken from GIT repository 
of Paul Stoffregen