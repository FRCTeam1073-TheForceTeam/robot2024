// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.PubSubOption;

public class Drivetrain extends Diagnostics{
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// class AprilTagSubscriber {
//   // the publisher is an instance variable so its lifetime matches that of the class
//   IntegerArraySubscriber intArraySub;
  
//   public void GetAprilTag(IntegerArrayTopic intArrayTopic) {
//       // start publishing; the return value must be retained (in this case, via
//       // an instance variable)
//       intArraySub = intArrayTopic.subscribe();
//     }
//   }