// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class CollectorSpeedCommand extends Command {

  Collector m_collector;
  double speed;

  /** Creates a new CollectorSpeedCommand. */
  public CollectorSpeedCommand(Collector collector, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collector;
    this.speed = speed;

    addRequirements(m_collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collector.setTargetCollectorVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
