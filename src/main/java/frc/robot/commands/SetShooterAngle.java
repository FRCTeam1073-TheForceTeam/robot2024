/* run pivot motor until at the right angle */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;


public class SetShooterAngle extends Command {
  private Shooter shooter;
  private Pivot pivot;
  double targetPositionRotations;
  /** Creates a new SetShooterAngle. */
  public SetShooterAngle(Shooter m_shooter, double targetPositionRotations) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = m_shooter;
    this.targetPositionRotations = targetPositionRotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setTargetPositionInRad(targetPositionRotations);
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
    return (pivot.getCurrentPositionInRad() >= targetPositionRotations * 0.98);
  }
}
