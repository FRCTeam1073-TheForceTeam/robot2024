// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class SetFeederVelocity extends Command {

  private Feeder m_feeder;
  private double m_speed;
  private boolean wait;

  /** Creates a new StopFeeder. */
  public SetFeederVelocity(Feeder feeder, double speed, boolean wait) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feeder = feeder;
    m_speed = speed;
    this.wait = wait;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.setTargetVelocityInMPS(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.setTargetVelocityInMPS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!wait) {
      return true;
    }
    return (m_feeder.getCurrentVelocityInMPS() >= m_speed * 0.98);  
  }
}
