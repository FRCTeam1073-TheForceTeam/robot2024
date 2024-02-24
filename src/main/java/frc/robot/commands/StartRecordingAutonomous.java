// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;

public class StartRecordingAutonomous extends Command {
  Camera camera;
  Boolean weAreFinished;
  int initcounter;
  int execcounter;
  /** Creates a new StartCamera. */
  public StartRecordingAutonomous(Camera camera) {
    this.camera = camera;
    addRequirements(camera);
    this.weAreFinished = false;
    this.initcounter = 0;
    this.execcounter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.weAreFinished = false;
    this.initcounter += 1;
    //System.out.println(String.format("StartRecordingAutonomous.java init counter: %s", this.initcounter));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.weAreFinished = false;
    System.out.println("starting autonomous recording");
    camera.startRecordingAutonomous();
    // TODO: listen for a reponse before finishing, ideally retry
    this.weAreFinished = true;
    this.execcounter += 1;
    //System.out.println(String.format("StartRecordingAutonomous.java exec counter: %s", this.execcounter));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("in StartRecordingAutonomous.java end(), setting weAreFinished to true");
    this.weAreFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.weAreFinished;
  }
}
