// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederAdjustCommand extends Command {
  Feeder feeder;
  private double maxFeedRange = 0.04;
  private double minFeedRange = 0.035; //0.03
  /** Creates a new AdjustFeed. */
  public FeederAdjustCommand(Feeder feeder) {
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(feeder.getTofRange() < minFeedRange){
      // if note is too close to ToF sensor, need to back out of the feeder (slowly)
      feeder.setTargetVelocityInMPS(-0.3);
    }
    if(feeder.getTofRange() > maxFeedRange){
      // if note is too far from the ToF sensor, pull farther into feeder (slowly)
      feeder.setTargetVelocityInMPS(0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // when in range, stop the feeder motor
    feeder.setTargetVelocityInMPS(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((feeder.getTofRange() <= maxFeedRange) && (feeder.getTofRange() >= minFeedRange));
  }
}
