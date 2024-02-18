// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;

public class StartRecordingAutonomous extends Command {
  Camera camera;
  Boolean weAreFinished;
  /** Creates a new StartCamera. */
  public StartRecordingAutonomous(Camera camera) {
    this.camera = camera;
    addRequirements(camera);
    this.weAreFinished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.weAreFinished = false;
    System.out.println("in StartRecordingAutonomous.java execute(), starting autonomous recording");
    camera.startRecordingAutonomous();
    // ideally listen for a reponse here, maybe retry
    this.weAreFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("in StartRecordingAutonomous.java end(), setting weAreFinished to true");
    this.weAreFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.weAreFinished;
  }
}
