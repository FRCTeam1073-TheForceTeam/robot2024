/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double shooterTopMPS;
  private double shooterBottomMPS;
  
  /* Creates a new RunShooter. */

  public RunShooter(Shooter shooter, double topShootorMPS, double bottomShooterMPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterTopMPS = topShootorMPS;
    this.shooterBottomMPS = bottomShooterMPS;
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setTargetTopVelocityInMPS(shooterTopMPS);
    shooter.setTargetBottomVelocityInMPS(shooterBottomMPS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.setTargetTopVelocityInMPS(0);
    // shooter.setTargetBottomVelocityInMPS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((shooter.getCurrentTopVelocityInMPS() >= (0.95 * shooterTopMPS)) && 
    (shooter.getCurrentBottomVelocityInMPS() >= (0.95 * shooterBottomMPS))){
      return true;
    }
    else{
      return false;
    }
  }
}
