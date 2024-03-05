/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * WaitForX is a utility command that succeeds when you pass a certain X value.
 */
public class WaitForX extends Command {
  private Drivetrain drivetrain;
  private double Xpos;
  private double posTolerance = 0.2;

  public WaitForX(Drivetrain drivetrain, double X, double posTolerance) {
    this.drivetrain = drivetrain;
    this.Xpos = X;
    this.posTolerance = posTolerance;

    // We just observe the drivetrain, we don't require it.
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
    if (posTolerance > 0.0 && drivetrain.getOdometry().getTranslation().getX() > Xpos)
      return true;
    else if (posTolerance <= 0.0 && drivetrain.getOdometry().getTranslation().getX() < Xpos)
      return true;
    else
      return false;
  }
}
