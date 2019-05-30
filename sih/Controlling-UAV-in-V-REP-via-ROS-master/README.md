# Controlling-UAV-in-V-REP-via-ROS
A UAV model in V-REP can be controlled using ROS commands in terms of pitch, roll, throttle and yaw.

Follow the steps to control UAV in V-REP using ROS
- Run **roscore** by typing the following command in your terminal:
```
roscore
```
- Launch the simulator by typing ```./vrep.sh``` in the V-REP directory and check if RosInterface is loaded properly
- Drag and drop **uav.ttm** in the scene
- On running simulation, you can find the following topics 
```/drone_command``` and ``` /drone_yaw```

“```/drone_command```” is a topic subscribed by the uav model. It commands the uav’s motion in terms of roll, pitch, yaw and throttle.
“```/drone_yaw```” is a topic published by the uav model. It indicates the uav’s orientation about the z-axis with respect to the V-REP world.
-Check the structure of the message of ```/drone_command``` using the following command:
```
rostopic type /drone_command | rosmsg show
```
- Check the structure of the message of ```/drone_yaw``` using the following command:
```
rostopic type /drone_yaw | rosmsg show
```


## Arming the UAV

The condition to arm the uav is ***rcThrottle = 1000*** (minimum value) and ***rcAUX4 ≥ 1300***. To test arming the uav model, publish the following message to the topic “```/drone_command```” by typing the command:
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1000, rcAUX1: 0, rcAUX2: 0, rcAUX3: 0, rcAUX4: 1500}"
```
This should now arm the uav. A message should pop on the V-REP window which says “ARMED” and the propellers should start rotating


## Flight (Take-Off)

The condition for the uav to take-off is ***rcThrottle ≥ 1500***, after arming. To test the uav’s take-off, publish the following message to increase the throttle:
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 0, rcAUX2: 0, rcAUX3: 0, rcAUX4: 1500}"
```
The uav should now steadily rise until a new command is given.

## Disarming the UAV

A disarmed uav means the uav is in a mode that will not take any commands from a user or software to fly.
The condition to disarm the uav is ***rcAUX4 ≤ 1200***. To test disarming the uav model, publish the following
message to the topic “```/drone_command```” by typing the command:
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500,rcPitch: 1500, rcYaw: 1500, rcThrottle: 1000, rcAUX1: 0, rcAUX2: 0, rcAUX3: 0, rcAUX4: 1200}"
```
The uav should now be disarmed. A message should pop on the V-REP window which says “DISARMED” and the propellers should stop rotating.

## Heading of the UAV
**Blue dummy** indicates the pitch direction of the uav
