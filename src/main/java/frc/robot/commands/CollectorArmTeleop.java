// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.OI;

public class CollectorArmTeleop extends Command {
  /** Creates a new CollectorArmTeleop. */
  CollectorArm arm;
  OI oi;

  public CollectorArmTeleop(CollectorArm arm, OI oi) {
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
    // double commandedAngle = arm.getTargetLiftAngle();
    // double commandedLength = arm.getTargetExtendLength();
    // targetLiftAngle = m_OI.getOperatorRightY();
    // targetExtendLength = m_OI.getOperatorLeftY();
    // commandedAngle = oi.getOperatorRightY();
    // commandedLength = oi.getOperatorLeftY();
    

    // // B Button
    // if(oi.getOperatorRawButton(2)) {
    //   arm.setTargetExtendLength(0);
    //   arm.setTargetLiftAngle(0);
    // }
    // // Y Button
    // if(oi.getOperatorRawButton(4)) {
    //   arm.setTargetExtendLength(-0.05);
    //   arm.setTargetLiftAngle(2);
    // }
    if(oi.getCollectMode()){
      double leftVelocity = oi.getOperatorLeftY();
      double rightVelocity = -oi.getOperatorRightY();
      if((Math.abs(leftVelocity) > 0.1) || (Math.abs(rightVelocity) > 0.1)){
        rightVelocity *= 0.8;
        leftVelocity *= 0.4;
        double currentAngle = arm.getCurrentLiftAngle();
        double currentExtend = arm.getCurrentExtendLength();
        arm.setTargetLiftAngleNotLimited(currentAngle + rightVelocity*.02);
        arm.setTargetExtendLengthNotLimited(currentExtend + leftVelocity*.02);
      }
    }
    if(oi.getOperatorViewButton()){
      arm.resetMotors();
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
