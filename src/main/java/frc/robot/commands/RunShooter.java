/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

public class RunShooter extends Command {
  private Shooter shooter;
  private Feeder feeder;
  private double shooterTopMPS;
  private double shooterBottomMPS;
  private double feederMotorMPS;
  
  /* Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double shootortopRPS, double shootorbottomRPS, double feederMotorMPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederMotorMPS = feederMotorMPS;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setTargetTopMotorVelocity(shooterTopMPS);
    shooter.setTargetBottomMotorVelocity(shooterBottomMPS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* if shooter motors are up to speed, then turn on trigger motors */
    /* might need to adjust the numbers depending on what % we want the power at */
    if ((shooter.getCurrentTopMotorVelocity() >= 0.98 * shooterTopMPS) && (shooter.getCurrentBottomMotorVelocity() >= 0.98 * shooterBottomMPS)){
      feeder.setTargetFeederMotorVelocity(feederMotorMPS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTargetTopMotorVelocity(0);
    shooter.setTargetBottomMotorVelocity(0);
    feeder.setTargetFeederMotorVelocity(0); 
  }

  // Returns true when the command should end.
  /* return true when the note is not in the trigger/shooter */
  @Override
  public boolean isFinished() {
    return !shooter.noteIsInShooter() && !feeder.noteIsInTrigger();
  }
}
