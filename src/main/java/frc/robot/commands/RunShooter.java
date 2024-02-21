/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double shooterTopMPS;
  private double shooterBottomMPS;
  
  /* Creates a new RunShooter. */

  public RunShooter(Shooter shooter, double shootortopRPS, double shootorbottomRPS) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setTargetTopVelocityInMPS(shooterTopMPS);
    shooter.setTargetBottomVelocityInMPS(shooterBottomMPS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* if shooter motors are up to speed, then turn on trigger motors */
    /* might need to adjust the numbers depending on what % we want the power at */
    // if ((shooter.getCurrentTopVelocityInMPS() >= 0.98 * shooterTopMPS) && (shooter.getCurrentBottomVelocityInMPS() >= 0.98 * shooterBottomMPS)){
    //   feeder.setTargetVelocityInMPS(feederMotorMPS);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  /* return true when the note is not in the trigger/shooter */
  @Override
  public boolean isFinished() {
    if((shooter.getCurrentTopVelocityInMPS() >= shooter.getTargetTopVelocityInMPS()) && 
    (shooter.getCurrentBottomVelocityInMPS() >= shooter.getTargetBottomVelocityInMPS())){
      return true;
    }
    else{
      return false;
    }
    
    //return !shooter.noteIsInShooter() && !feeder.noteIsInTrigger();
  }
}
