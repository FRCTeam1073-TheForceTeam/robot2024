// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.CollectorArm.POSE;

public class ArmPoseTeleop extends Command {
  /** Creates a new ArmPoseCommand. */
  CollectorArm arm;
  OI oi;

  public ArmPoseTeleop(CollectorArm arm, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.oi = oi;
    addRequirements(arm);
  }

  public SequentialCommandGroup startPose() {
    return new SequentialCommandGroup(
      new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE, 0.035, 0.015),
      new ArmPoseCommand(arm, POSE.START)
    );
  }

  // public SequentialCommandGroup stowIntermediatePose() {
  //   return new SequentialCommandGroup(
  //     new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE, 0.01, 0.02),
  //     new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE_2)

  //   );
  // }

  public SequentialCommandGroup stowPose() {
    return new SequentialCommandGroup(
      new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE, 0.025, 0.02),
      new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE_2, 0.01, 0.03),
      new ArmPoseCommand(arm, POSE.STOW)
    );
  }

  public SequentialCommandGroup ampPose() {
    return new SequentialCommandGroup(
      new ArmPoseCommand(arm, POSE.STOW_INTERMEDIATE, 0.055, 0.02),
      new ArmPoseCommand(arm, POSE.AMP)
    );
  }
}
