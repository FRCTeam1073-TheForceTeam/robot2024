// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.ShooterInterpolatorTable;

public class PivotRangeCommand extends Command 
{
  /** Creates a new SetPivotCommand. */
  private Pivot pivot;
  private RangeFinder rangefinder;
  private ShooterInterpolatorTable pivotTable;
  private double targetPositionRad;
  private boolean isPivotOn;

  double currentRange;
  double avgRange;

  double range;

  double count;

  public PivotRangeCommand(Pivot pivot, RangeFinder rangefinder) 
  {
    // when we do not have a set range to shoot from
    this.pivot = pivot;
    this.rangefinder = rangefinder;
    pivotTable = new ShooterInterpolatorTable();
    range = -1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  public PivotRangeCommand(Pivot pivot, double range)
  {
    // when we have a set range to shoot from
    this.pivot = pivot;
    this.range = range;
    pivotTable = new ShooterInterpolatorTable();
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPositionRad = pivotTable.interpolatePivotAngle(rangefinder.getRange());
    //targetPositionRad = pivot.getDebugPivotAngle();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (range != -1)
    {
      targetPositionRad = pivotTable.interpolatePivotAngle(range);
      pivot.setTargetPositionInRad(targetPositionRad);
    }
    else
    {
      if (count < 20)
      {
        currentRange = rangefinder.getRange();
    
        avgRange = (0.5 * avgRange) + (0.5 * currentRange);
        count++;
      }
      else
      {
        targetPositionRad = pivotTable.interpolatePivotAngle(avgRange);
        pivot.setTargetPositionInRad(targetPositionRad);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setTargetPositionInRad(targetPositionRad);
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    var error = Math.abs(pivot.getCurrentPositionInRad() - targetPositionRad);
    return (error <= 0.05);
  }
}
