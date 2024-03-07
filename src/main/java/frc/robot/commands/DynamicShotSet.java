// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class DynamicShotSet extends Command {
  private Shooter shooter;
  private Pivot pivot;
  private Drivetrain drivetrain;
  private double shooterVel;
  private double pivotAngle;
 
  /** Creates a new DynamicShotSet. */
  public DynamicShotSet(Shooter shooter, Pivot pivot, Drivetrain drivetrain, double shooterVel, double pivotAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pivot = pivot;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetTopVelocityInMPS(shooterVel);
    shooter.setTargetBottomVelocityInMPS((shooterVel - (drivetrain.getChassisSpeeds().vxMetersPerSecond)));
    pivot.setTargetPositionInRad(pivotAngle);
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
