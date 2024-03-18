/* run pivot motor until at the right angle */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class NWSetShooterVel extends Command {
  private Shooter shooter;
  double targetTopVel;
  double targetBottomVel;
  
  /** Creates a new SetShooterAngle. */
  public NWSetShooterVel(Shooter m_shooter, double targetTopVel, double targetBottomVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = m_shooter;
    this.targetTopVel = targetTopVel;
    this.targetBottomVel = targetBottomVel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    shooter.setTargetBottomVelocityInMPS(targetBottomVel);
    shooter.setTargetTopVelocityInMPS(targetTopVel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
