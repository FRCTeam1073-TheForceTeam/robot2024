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
  Timer timer;
  Boolean weAreFinished;
  public GetAprilTagInfo(SerialComms serialComms, Camera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.serialComms = serialComms;
    this.weAreFinished = false;
    addRequirements(camera, serialComms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    serialComms.send("1,ap");
    System.out.println("sent for april tag");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    serialComms.receive();
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
