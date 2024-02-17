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
    

    // B Button
    if(oi.getOperatorRawButton(2)) {
      //arm.setTargetLiftAngle(0);
      //arm.setTargetExtendLength(0);
      arm.setTargetLiftAngle(0);
    }
    // Y Button
    if(oi.getOperatorRawButton(4)) {
      //arm.setTargetLiftAngle(-0.75);
      //arm.setTargetExtendLength(0.03);
      arm.setTargetLiftAngle(2);

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
