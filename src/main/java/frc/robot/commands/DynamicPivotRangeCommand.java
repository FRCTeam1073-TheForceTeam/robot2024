// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.ShooterInterpolatorTable;

public class DynamicPivotRangeCommand extends Command {
  /** Creates a new DynamicPivotRangeCommand. */
  private Pivot pivot;
  private RangeFinder rangefinder;
  private ShooterInterpolatorTable pivotTable;
  private Drivetrain drivetrain;
  private AprilTagFinder tagFinder;
  private double targetPositionRad;
  private boolean isPivotOn;
  private double tolerance;
  private double defaultRange;

  double currentRange;
  double avgRange;

  double currentTime = 0.0;
  double oldTime = 0.0;

  double range;
  double rangeRate = 0;

  double count;

  double ySpeedProportional = 0;

  boolean isWaiting = true;


  public DynamicPivotRangeCommand(Pivot pivot, RangeFinder rangefinder, Drivetrain drivetrain, AprilTagFinder tagFinder) {
    this.pivot = pivot;
    this.rangefinder = rangefinder;
    this.drivetrain = drivetrain;
    this.tagFinder = tagFinder;
    pivotTable = new ShooterInterpolatorTable();
    tolerance = -1;
    range = -1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isWaiting = true;
    avgRange = 0.0;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isWaiting = true;
  
    if (rangefinder.getRange() > 1.0)
    {
      currentRange = rangefinder.getRange();
    }
    
    ySpeedProportional = (drivetrain.getChassisSpeeds().vyMetersPerSecond * 0.3);
    
    if(ySpeedProportional > 0){
      ySpeedProportional += drivetrain.getChassisSpeeds().vyMetersPerSecond * 0.15;
    }

    avgRange = (0.25 * avgRange) + (0.75 * currentRange);
    
    if (count < 20)
    {
      count++;
    }
    else{ 
      targetPositionRad = pivotTable.interpolatePivotAngle(avgRange) - ySpeedProportional;
      pivot.setTargetPositionInRad(targetPositionRad);
    }

    
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
