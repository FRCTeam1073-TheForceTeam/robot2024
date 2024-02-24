// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorArm.POSE;

public class ArmPoseCommand extends Command {

  CollectorArm arm;
  POSE pose;

  double targetLift;
  double liftTolerence = 0.01;

  /** Creates a new ArmPoseCommand. */
  public ArmPoseCommand(CollectorArm arm, POSE pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (pose) {
      case START:
        targetLift = 0.0;
        break;
      case STOW:
        targetLift = 0.2;
        break;
      case HANDOFF:
        targetLift = 0.0;
        break;
      case AMP:
        targetLift = 1.9453125;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setTargetLiftAngle(targetLift);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(arm.getCurrentLiftAngle() - targetLift);
    if(error < liftTolerence){
      return true;
    }
    else{
      return false;
    }
  }
}
