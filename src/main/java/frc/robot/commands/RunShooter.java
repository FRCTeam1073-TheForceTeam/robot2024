/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterpolatorTable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class RunShooter extends Command {
  private Shooter shooter;
  private ShooterInterpolatorTable shooterInterpolatorTable;
  private double shooterTopMPS;
  private double shooterBottomMPS;
  private double range;
  private double velocityMPS;
  
  /* Creates a new RunShooter. */

  public RunShooter(Shooter shooter, double velocityMPS) {
    /* Range to calculate the speed needed */
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.range = range;
    this.velocityMPS = velocityMPS;
    this.shooterInterpolatorTable = shooterInterpolatorTable;
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    // shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(range);
    // shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(range);
    shooterTopMPS = velocityMPS;
    shooterBottomMPS = velocityMPS;
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
    if((shooter.getCurrentTopVelocityInMPS() >= (0.98 * shooterTopMPS)) && 
    (shooter.getCurrentBottomVelocityInMPS() >= (0.98 * shooterBottomMPS))){
      return true;
    }
    else{
      return false;
    }
  }
}
