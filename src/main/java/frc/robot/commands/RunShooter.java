/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterpolatorTable;


public class RunShooter extends Command {
  private Shooter shooter;
  private ShooterInterpolatorTable shooterInterpolatorTable;
  private RangeFinder rangefinder;
  private double shooterTopMPS;
  private double shooterBottomMPS;

  double averageBottomVel = 0;
  double averageTopVel = 0;

  double currentBottomVel;
  double currentTopVel;

  double range = 0;

  double count;
  
  /* Creates a new RunShooter. */

  public RunShooter(Shooter shooter, RangeFinder rangeFinder) {
    /* Range to calculate the speed needed */
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.rangefinder = rangeFinder;
    this.shooterInterpolatorTable = new ShooterInterpolatorTable();
    range = -1;
    addRequirements(shooter);
  }

  public RunShooter(Shooter shooter, double range)
  {
    // for using the command when shooting from points with known ranges

    this.shooter = shooter;
    this.range = range;
    this.shooterInterpolatorTable = new ShooterInterpolatorTable();
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() 
  {
    if (range != -1) // if using rangefinder
    {
      shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
      shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
    }
    else // if using a set range
    {
      shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(range);
      shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(range);
    }
    
    // shooterTopMPS = shooter.getRunShooterTargetBottomVelocityInMPS();
    // shooterBottomMPS = shooter.getRunShooterTargetBottomVelocityInMPS();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentBottomVel = shooter.getCurrentBottomVelocityInMPS();
    currentTopVel = shooter.getCurrentTopVelocityInMPS();
    
    shooter.setTargetTopVelocityInMPS(shooterTopMPS);
    shooter.setTargetBottomVelocityInMPS(shooterBottomMPS);

    averageBottomVel = (0.5 * averageBottomVel) + (0.5 * currentBottomVel);
    averageTopVel = (0.5 * averageTopVel) + (0.5 * currentTopVel);

    if((Math.abs(averageBottomVel - shooterBottomMPS) < 0.55) && (Math.abs(averageTopVel - shooterTopMPS) < 0.55)){
      count++;
    }
    else if(count > 0){
      count--;
    }
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
    // if((shooter.getCurrentTopVelocityInMPS() >= (0.98 * shooterTopMPS)) && 
    // (shooter.getCurrentBottomVelocityInMPS() >= (0.98 * shooterBottomMPS))){
    //   return true;
    // }
    // else{
    //   return false;
    // }
    if(count > 20){
      return true;
    }
    else{
      return false;
    }
  }
}
