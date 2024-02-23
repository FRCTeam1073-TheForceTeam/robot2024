# robot2024

Team 1073's robot code for the 2024 FRC Season: Crecendo

## Can ID

The primary devices on CANBus are four swerve modules /w CANCoder feedback, a Pigeon 2 IMU for orientation, and 2-3 motors /w CANCoder feedback for the arm. Possible additional nodes to come soon.
Swerve Module #0 is front left
Swerve Module #1 is front right
Swerve Module #2 is back left
Swerve Module #3 is back right

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


## PS4 Controller Symbols

| Letter | Shape    |
| ------ | -------- |
|   A    |   X      |
|   B    | Circle   |
|   X    | Square   |
|   Y    | Triangle |
