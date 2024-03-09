// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SerialComms;

public class GetAprilTagInfo extends Command {
  /** Creates a new getApriltTagInfo. */
  SerialComms serialComms;
  Camera camera;
  String tagID;
  Timer timer;
  Boolean weAreFinished;
  public GetAprilTagInfo(SerialComms serialComms, Camera camera, String tagID) {
    System.out.println("in GetAprilTagInfo command");
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.serialComms = serialComms;
    this.tagID = tagID;
    this.weAreFinished = false;
    addRequirements(camera, serialComms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String aprilInfo = camera.getAprilTagInfo(this.tagID);
    System.out.println("sent for april tag");
    System.out.println(aprilInfo);
    this.weAreFinished = true;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.weAreFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.weAreFinished;
  }
}