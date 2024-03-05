/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * The utility command waits (first part of a sequnce) until you get 'close enough'
 * to a given pose, as defined by tolerances.
 */
public class WaitForPoint extends Command {
  private Drivetrain drivetrain;
  private Pose2d point;
  private double posTolerance = 0.2;
  private double angTolerance = 0.2;
  
  public WaitForPoint(Drivetrain drivetrain, Pose2d point, double posTolerance, double angTolerance) {
    this.drivetrain = drivetrain;
    this.point = point;
    this.posTolerance = posTolerance;
    this.angTolerance = angTolerance;
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // This command does nothing.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // This command does nothing.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Transform2d delta = point.minus(drivetrain.getOdometry());

    if (delta.getTranslation().getNorm() < posTolerance && delta.getRotation().getRadians() < angTolerance)
      return true;
    else
      return false;
  }
}
