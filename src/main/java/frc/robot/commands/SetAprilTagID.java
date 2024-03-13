// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;

public class SetAprilTagID extends Command {
  String tagID = "-1";
  AprilTagFinder aprilTagFinder;
  public SetAprilTagID(AprilTagFinder aprilTagFinder, String tagID) {
    this.tagID = tagID;
    this.aprilTagFinder = aprilTagFinder;
    addRequirements(aprilTagFinder);
  }

  @Override
  public void execute() {
    //this.aprilTagFinder.tagID = this.tagID;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
