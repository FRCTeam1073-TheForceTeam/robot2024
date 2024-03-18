/* until it passes the beam breaker */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterInterpolatorTable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class NWStopShooter extends Command {
  private Shooter shooter;
  private double minRange;
  private double maxRange;
  private boolean noteEntered;
  private boolean noteGone;
  
  /* Creates a new StopShooter. */

  public NWStopShooter(Shooter shooter) {
    /* Range to calculate the speed needed */
    // Use addRequirements() here to declare subsystem dependencies.
    maxRange = 0.4;
    minRange = 0.4;
    this.shooter = shooter;
    noteEntered = false;
    noteGone = false;
    addRequirements(shooter);
  }

// Called when the command is initially scheduled.
  /* start shooter wheels to get them up to speed */
  @Override
  public void initialize() {
    shooter.setShot(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getTofRange() < minRange){
      noteEntered = true;
    }
    if(noteEntered && shooter.getTofRange() > maxRange){
      noteGone = true;
    }
    if(noteEntered && noteGone){
      shooter.setShot(true);
      shooter.setTargetTopVelocityInMPS(0);
      shooter.setTargetBottomVelocityInMPS(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setCommandedToShoot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (noteEntered && noteGone)
    {
        return true;
    }
    return false;
  }
}
