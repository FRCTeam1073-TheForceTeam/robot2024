// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;


public class SpeakerTagAllianceSearch extends Command {
  AprilTagFinder finder;
  /** Creates a new SpeakerTagAllianceSearch. */
  public SpeakerTagAllianceSearch(AprilTagFinder finder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.finder = finder;

    addRequirements(finder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().isPresent()){
      if (DriverStation.getAlliance().get() == Alliance.Blue){
        finder.setSearchTagId(7);
      }else{
        finder.setSearchTagId(4);
    }
  }
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

