/* run motor until note hits beam break */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.OI;


public class LoadFeeder extends Command {
  private Feeder feeder;
  private OI oi;

  double minRange;
  double maxRange;
  double currentTofRange;
  double oldTofRange;
  double feederRate;
  double intakeFeederRateThreshold = 0.001;
  
  /** Creates a new LoadTrigger. */
  public LoadFeeder(Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feeder = feeder;
    this.oi = oi;
    addRequirements(feeder);
    minRange = 0.35;
    maxRange = 0.8;
    currentTofRange = 0;
    oldTofRange = 0;
    feederRate = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTofRange = feeder.getTofRange();
    feederRate = ((currentTofRange - oldTofRange) / 0.02);
    oldTofRange = currentTofRange;
    feeder.setTargetVelocityInMPS(10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setTargetVelocityInMPS(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (feeder.getTofRange() < minRange);
    //return (feederRate < intakeFeederRateThreshold);
  }
}
