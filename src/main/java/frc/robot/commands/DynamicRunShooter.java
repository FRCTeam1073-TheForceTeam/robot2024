// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterpolatorTable;

public class DynamicRunShooter extends Command {
  /** Creates a new DynamicRunShooter. */
  private Shooter shooter;
  private ShooterInterpolatorTable shooterInterpolatorTable;
  private RangeFinder rangefinder;
  private double shooterTopMPS;
  private double shooterBottomMPS;
  private double currentRange;
  private double avgRange;

  double averageBottomVel = 0;
  double averageTopVel = 0;

  double currentBottomVel;
  double currentTopVel;

  double range = 0;

  double count;

  public DynamicRunShooter(Shooter shooter, RangeFinder rangeFinder) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.rangefinder = rangeFinder;
    this.shooterInterpolatorTable = new ShooterInterpolatorTable();

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setCommandedToShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (rangefinder.getRange() > 1.0)
    {
      currentRange = rangefinder.getRange();
    }
  
    avgRange = (0.2 * avgRange) + (0.8 * currentRange);
    
    // if (count < 20)
    // {
    //   count++;
    // }
    // else{ 
      shooterTopMPS = shooterInterpolatorTable.interpolateShooterVelocity(avgRange);
      shooterBottomMPS = shooterInterpolatorTable.interpolateShooterVelocity(avgRange);

      shooter.setTargetTopVelocityInMPS(shooterTopMPS);
      shooter.setTargetBottomVelocityInMPS(shooterBottomMPS);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
