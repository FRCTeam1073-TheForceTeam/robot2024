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
  private Feeder trigger;
  private double shooterTopRPS;
  private double shooterBottomRPS;
  private double triggerTopRPS;
  private double triggerBottomRPS;
  /* Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double shootortopRPS, double shootorbottomRPS, double triggerTopRPS, double triggerBottomRPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterTopRPS = shooterTopRPS;
    this.shooterBottomRPS = shooterBottomRPS;
    this.triggerTopRPS = triggerTopRPS;
    this.triggerBottomRPS = triggerBottomRPS; 
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setTopShooterMotorVelocity(shooterTopRPS);
    shooter.setBottomShooterMotorVelocity(shooterBottomRPS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* if shooter motors are up to speed, then turn on trigger motors */
    /* might need to adjust the numbers depending on what % we want the power at */
    if ((shooter.getTopShooterMotorVelocity() >= 0.98 * shooterTopRPS) && (shooter.getBottomShooterMotorVelocity() >= 0.98 * shooterBottomRPS)){
      trigger.setTopTriggerMotorVelocity(triggerTopRPS);
      trigger.setBottomTriggerMotorVelocity(triggerBottomRPS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTopShooterMotorVelocity(0);
    shooter.setBottomShooterMotorVelocity(0);
    trigger.setTopTriggerMotorVelocity(0);
    trigger.setBottomTriggerMotorVelocity(0);    
  }

  // Returns true when the command should end.
  /* return true when the note is not in the trigger/shooter */
  @Override
  public boolean isFinished() {
    return !shooter.noteIsInShooter() && !trigger.noteIsInTrigger();
  }
}
