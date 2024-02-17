// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.CollectorArm.POSE;

public class ArmPoseCommand extends Command {
  /** Creates a new ArmPoseCommand. */
  CollectorArm arm;
  OI oi;

  public ArmPoseCommand(CollectorArm arm, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.oi = oi;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // A Button: START
    if(oi.getOperatorRawButton(1)) {
      arm.setPoseName(POSE.START);
      arm.setTargetExtendLength(0);
      arm.setTargetLiftAngle(0);
    }

    // B Button STOW (lift)
    if(oi.getOperatorRawButton(2)) {
      arm.setTargetExtendLength(0);
      arm.setTargetLiftAngle(0.23315429);
    }

    // X Button STOW (extend)
    if(oi.getOperatorRawButton(3)) {
      arm.setPoseName(POSE.STOW);
      arm.setTargetExtendLength(0.1047363281);
      arm.setTargetLiftAngle(0.23315429);
    }

    // Y Button AMP
    if(oi.getOperatorRawButton(4)) {
      arm.setPoseName(POSE.AMP);
      arm.setTargetExtendLength(-0.04565429687);
      arm.setTargetLiftAngle(1.98388671875);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
