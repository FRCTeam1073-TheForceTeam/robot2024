// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.NoteFinder;
import frc.robot.subsystems.OI;

public class AlignToNote extends MotionSchema {
  NoteFinder noteFinder;
  Headlight headlight;
  OI oi;
  double noteRotation;
  PIDController turnController = new PIDController(0.15, 0, 0.015);

  public AlignToNote(NoteFinder noteFinder, Headlight headlight, OI oi) 
  {
    this.noteFinder = noteFinder;
    this.headlight = headlight;
    this.oi = oi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) {
    headlight.setHeadlight(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void update(Drivetrain drivetrain) 
  {
    var note = noteFinder.getCurrentNoteData();

    if (oi.getDriverBButton())
    {
      headlight.setHeadlight(true);
      if (note.validNote())
      {
       //rotation = (0.02 + (drivetrain.getChassisSpeeds().vyMetersPerSecond * 0.01)) * (0 - apriltag.yaw);
       noteRotation = turnController.calculate(note.noteYaw,  0);
       SmartDashboard.putNumber("turnController-rotation", noteRotation);
       MathUtil.clamp(noteRotation, -1.5, 1.5);
        setRotate(noteRotation, 1.0);
      }
      else //if((Math.abs(160 - apriltag.cx) < 20) && !finder.tagFound())
      {
        setRotate(0.0, 0.0);
      }
    }
    else
    {
      setRotate(0.0, 0.0);
      headlight.setHeadlight(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    headlight.setHeadlight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}