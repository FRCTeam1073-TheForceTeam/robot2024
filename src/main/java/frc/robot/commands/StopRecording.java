// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;

public class StopRecording extends Command {
  Camera[] cameras;
  /** Creates a new StopCamera. */
  public StopRecording(Camera[] cameras) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cameras = cameras;
    addRequirements(cameras);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("in StopRecording.java initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("in StopRecording.java exec()");
    for (int i = 0; i < cameras.length; i++){

    // cameras[i].stopRecording();
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: see if this works without the this.weAreFinished business
    //System.out.println("in StopRecording.java isFinished()");
    return true;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
