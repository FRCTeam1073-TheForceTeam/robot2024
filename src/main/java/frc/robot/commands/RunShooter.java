/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double shooterTopRPS;
  private double shooterBottomRPS;
  private double feederTopRPS;
  private double feederBottomRPS;
  /* Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double shootortopRPS, double shootorbottomRPS, double feederTopRPS, double feederBottomRPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterTopRPS = shooterTopRPS;
    this.shooterBottomRPS = shooterBottomRPS;
    this.feederTopRPS = feederTopRPS;
    this.feederBottomRPS = feederBottomRPS; 
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setTopMotorVelocity(shooterTopRPS);
    shooter.setBottomMotorVelocity(shooterBottomRPS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* if shooter motors are up to speed, then turn on trigger motors */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTopMotorVelocity(0);
    shooter.setBottomMotorVelocity(0);
    shooter.setTriggerMotorTopVelocity(0);
   shooter.setTriggerMotorBottomVelocity(0);    
  }

  // Returns true when the command should end.
  /* return true when the note is not in the trigger/shooter */
  @Override
  public boolean isFinished() {
    return !shooter.noteIsInShooter() && !shooter.noteIsInTrigger();
  }
}
