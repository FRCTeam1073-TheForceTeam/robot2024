// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class SetPivotCommand extends Command {
  /** Creates a new SetPivotCommand. */
  private Pivot pivot;
  private double targetPositionRad;
  private boolean isPivotOn;

  public SetPivotCommand(Pivot pivot, double pivotAngle) {
    this.pivot = pivot;
    targetPositionRad = pivotAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setTargetPositionInRad(targetPositionRad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return (Math.abs(pivot.getCurrentPositionInRad()) >= Math.abs((0.98 * targetPositionRad)));
  }
}
