# robot2024

Team 1073's robot code for the 2024 FRC Season: Crecendo

## Can ID

The primary devices on the CANBus are 4 swerve modules with CANCoder feedback, a Pigeon 2 IMU for orientation, 3 motors for the collector and arm, 5 motors for the shooter/feeder, and 1 motor for the climber.

Swerve module #0 is front left, #1 is front right, #2 is back left, and #3 is back right.

| Device                | CAN ID |
| --------------------- | ------ |
| PDH                   | 0      |
| Swerve #0 Encoder     | 1      |
| Swerve #1 Encoder     | 2      |
| Swerve #2 Encoder     | 3      |
| Swerve #3 Encoder     | 4      |
| Swerve #0 Steer Motor | 5      |
| Swerve #1 Steer Motor | 6      |
| Swerve #2 Steer Motor | 7      |
| Swerve #3 Steer Motor | 8      |
| Swerve #0 Drive Motor | 9      |
| Swerve #1 Drive Motor | 10     |
| Swerve #2 Drive Motor | 11     |
| Swerve #3 Drive Motor | 12     |
| Pigeon 2              | 13     |
| Collector Motor       | 14     |
| Lift Motor            | 15     |
| Extend Motor          | 16     |
| Top Shooter Motor     | 17     |
| Bottom Shooter Motor  | 18     |
| Leader Feeder Motor   | 19     |
| Follower Feeder Motor | 20     |
| Pivot Motor           | 21     |
| Climber Motor #1      | 22     |

| Digital IO            | Port   |
| --------------------- | ------ |
| Collector Tof         | 0      |
| Feeder Tof            | 1      |
| Shooter Beam Break    | 2      |

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
