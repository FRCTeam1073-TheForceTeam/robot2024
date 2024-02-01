// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.OI;

public class ShooterTestCommand extends Command {
  /** Creates a new ShooterTestCommand. */
  private Shooter shooter;
  private OI oi;
  private double topShooterMotorVelocity;
  private double bottomShooterMotorVelocity;
  private double triggerMotorLeaderVelocity;
  private double triggerMotorFollowerVelocity;
  public ShooterTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topShooterMotorVelocity = SmartDashboard.getNumber("Top Shooter Motor Velocity", 0.0); //in RPS
    bottomShooterMotorVelocity = SmartDashboard.getNumber("Bottom Shooter Motor Velocity", 0.0); //in RPS
    triggerMotorLeaderVelocity = SmartDashboard.getNumber("Trigger Motor Leader Velocity", 0.0); //in RPS
    triggerMotorFollowerVelocity = SmartDashboard.getNumber("Trigger Motor Follower Velocity", 0.0); //in RPS
    
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
