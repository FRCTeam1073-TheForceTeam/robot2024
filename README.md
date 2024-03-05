# robot2024

Team 1073's robot code for the 2024 FRC Season: Crecendo

## Direction Conventions

1. Collector movement is defined as meters/second in X axis (front of robot) direction. Therefore sucking a note into the robot is a negative collect velocity, spitting a note out is a positive collect velocity.
2. Collector arm extension is defined as meters away from the pivot of the collector. This means that sticking the arm out further is positive meters and pulling it in closer is negative meters from the zero position.
3. Collector arm rotation is defined based on rotation about the +Y axis of the robot chassis. It follows the right hand rule. Therefore, "raising up" the collector is a negative rotation about the Y axis of the robot and "lowering" the collector is a positive rotation about the Y axis.
4. The launcher pivot is likewise defined based on rotation about the +Y axis of the robot chassis. It follows the right hand rule. This is based on where the launcher is aimed, so the aiming vector of the collector is moved upwards by negative rotation about the +Y axis and is moved downwards by a positive rotation about the +Y axis. 
5. The launcher/trigger rollers are scaled to meters and/or meters/second and positive movement is moving the note along/out of the robot and negative movement would be backwards along the launcher/into the robot.

## Can ID

The primary devices on the CANBus are 4 swerve modules with CANCoder feedback, a Pigeon 2 IMU for orientation, 3 motors for the collector and arm, 5 motors for the shooter/feeder, and 1 motor for the climber.

Swerve module #0 is front left, #1 is front right, #2 is back left, and #3 is back right.

| Device                | CAN ID |   BUS    |
| --------------------- | ------ | -------- |
| PDH                   |   0    |   rio    |
| Swerve #0 Encoder     |   1    |   rio    |
| Swerve #1 Encoder     |   2    |   rio    |
| Swerve #2 Encoder     |   3    |   rio    |
| Swerve #3 Encoder     |   4    |   rio    |
| Swerve #0 Steer Motor |   5    |   rio    |
| Swerve #1 Steer Motor |   6    |   rio    |
| Swerve #2 Steer Motor |   7    |   rio    |
| Swerve #3 Steer Motor |   8    |   rio    |
| Swerve #0 Drive Motor |   9    |   rio    |
| Swerve #1 Drive Motor |  10    |   rio    |
| Swerve #2 Drive Motor |  11    |   rio    |
| Swerve #3 Drive Motor |  12    |   rio    |
| Pigeon 2              |  13    |   rio    |
| Collector Motor       |  14    | CANivore |
| Lift Motor            |  15    | CANivore |
| Extend Motor          |  16    | CANivore |
| Top Shooter Motor     |  17    | CANivore |
| Bottom Shooter Motor  |  18    | CANivore |
| Leader Feeder Motor   |  19    | CANivore |
| Follower Feeder Motor |  20    | CANivore |
| Pivot Motor           |  21    | CANivore |
| Climber Motor #1      |  22    | CANivore |

| Digital IO            | Port   |
| --------------------- | ------ |
| Collector Tof         |   0    |
| Feeder Tof            |   1    |
| Shooter Tof           |   2    |



## Bling slots
(will be added later in the season)

| Slot | Mechanism |
| ---- | --------- |

## XBox Controller Buttons

### Driver Controller

| Button                   | Number | Function                   |
| ------------------------ | ------ | -------------------------- |
| A                        | 1      | Reset Odometry             |
| B                        | 2      |                            |
| X                        | 3      |                            |
| Y                        | 4      |                            |
| Front Left Trigger       | 5      | Parking Brake              |
| Front Right Trigger      | 6      |                            |
| View Button              | 7      | Field Centric Toggle       |
| Menu Button              | 8      | Zero Heading               |
| Push Down Left Joystick  | 9      |                            |
| Push Down Right Joystick | 10     |                            |

| Axes & Other             | Axes   | Function                   |
| ------------------------ | ------ | -------------------------- |
| Left Joystick            | 0 & 1  | Translate                  |
| Right Joystick           | 2      | Rotate                     |
| Back Left Trigger        | 3      | Increase Speed             |
| Back Right Trigger       | 4 & 5  | Increase Speed             |
| Dpad                     |        |                            |

### Operator Controller

(W0) = temporary mechanism control for week zero

|          Button          | Number | Function                   |
| ------------------------ | ------ | -------------------------- |
| A                        | 1      |                            |
| B                        | 2      |                            |
| X                        | 3      | (W0) Collector             |
| Y                        | 4      |                            |
| Front Left Trigger       | 5      |                            |
| Front Right Trigger      | 6      |                            |
| View Button              | 7      |                            |
| Menu Button              | 8      |                            |
| Push Down Left Joystick  | 9      |                            |
| Push Down Right Joystick | 10     |                            |

| Axes & Other             | Axes   | Function                   |
| ------------------------ | ------ | -------------------------- |
| Left Joystick            | 0 & 1  |                            |
| Right Joystick           | 2      |                            |
| Back Left Trigger        | 3      |                            |
| Back Right Trigger       | 4 & 5  |                            |
| Dpad                     |        |                            |

## Reference

| Xbox Letter | PS4 Shape    |
| ----------- | ------------ |
| A           | X            |
| B           | Circle       |
| X           | Square       |
| Y           | Triangle     |
