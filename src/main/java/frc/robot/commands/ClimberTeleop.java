// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;

public class ClimberTeleop extends Command {

  Climber m_climber;
  OI m_oi;

  /** Creates a new ClimberTeleop. */
  public ClimberTeleop(Climber climber, OI oi) {
    m_climber = climber;
    m_oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftVelocity = -m_oi.getOperatorLeftY();
    if(Math.abs(leftVelocity) < 0.05){
      leftVelocity = 0;
    }
    double rightVelocity = -m_oi.getOperatorRightY();
    if(Math.abs(rightVelocity) < 0.05){
      rightVelocity = 0;
    }

    rightVelocity *= 0.1;
    leftVelocity *= 0.1;

    m_climber.setVelocities(leftVelocity, rightVelocity);
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