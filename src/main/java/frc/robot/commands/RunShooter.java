/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterpolatorTable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


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

  double currentRange;
  double avgRange;

  double velCount;

  double rangeCount;
  
  /* Creates a new RunShooter. */

  public RunShooter(Shooter shooter, RangeFinder rangeFinder) {
    /* Range to calculate the speed needed */
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.rangefinder = rangeFinder;
    this.shooterInterpolatorTable = new ShooterInterpolatorTable();
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    // shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
    // shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
    shooterTopMPS = shooter.getRunShooterTargetBottomVelocityInMPS();
    shooterBottomMPS = shooter.getRunShooterTargetBottomVelocityInMPS();
    velCount = 0;
    rangeCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if(rangeCount < 20){
    //   currentRange = rangefinder.getRange();
    
    //   avgRange = (0.5 * avgRange) + (0.5 * currentRange);
    //   rangeCount++;
    // }
    // else if(rangeCount == 20){
    //   shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
    //   shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(rangefinder.getRange());
    //   rangeCount++;
    // }


    currentBottomVel = shooter.getCurrentBottomVelocityInMPS();
    currentTopVel = shooter.getCurrentTopVelocityInMPS();
    
    shooter.setTargetTopVelocityInMPS(shooterTopMPS);
    shooter.setTargetBottomVelocityInMPS(shooterBottomMPS);

    averageBottomVel = (0.5 * averageBottomVel) + (0.5 * currentBottomVel);
    averageTopVel = (0.5 * averageTopVel) + (0.5 * currentTopVel);

    if((Math.abs(averageBottomVel - shooterBottomMPS) < 0.5) && (Math.abs(averageTopVel - shooterTopMPS) < 0.5)){
      velCount++;
    }
    else if(velCount > 0){
      velCount--;
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
    if(velCount > 20){
      return true;
    }
    else{
      return false;
    }
  }
}
