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
  double previousLeftPosition = 0.0;
  double previousRightPosition = 0.0;

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
    if(!m_oi.getCollectMode()){
      double leftVelocity = -m_oi.getOperatorLeftY();
      if(Math.abs(leftVelocity) < 0.1){
        leftVelocity = 0;
      }
      double rightVelocity = -m_oi.getOperatorRightY();
      if(Math.abs(rightVelocity) < 0.1){
        rightVelocity = 0;
      }

      rightVelocity *= 0.3;
      leftVelocity *= 0.3;

      m_climber.setVelocities(leftVelocity, rightVelocity);
    }
    if(m_oi.getCollectMode()){
      if(Math.abs(m_climber.getLeftPosition() - previousLeftPosition) > .04 ||
      Math.abs(m_climber.getRightPosition() - previousRightPosition) > .04){
        m_climber.setPositions(m_climber.getLeftPosition(), m_climber.getRightPosition());
      }
      previousLeftPosition = m_climber.getLeftPosition();
      previousRightPosition = m_climber.getRightPosition();
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
