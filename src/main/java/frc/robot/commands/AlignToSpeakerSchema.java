// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Headlight;

public class AlignToSpeakerSchema extends MotionSchema 
{
  AprilTagFinder finder;
  Headlight headlight;
  OI oi;
  double rotation;
  PIDController turnController = new PIDController(0.15, 0, 0.015);

  /** Creates a new AlignToSpeakerSchema. */
  public AlignToSpeakerSchema(AprilTagFinder finder, Headlight headlight, OI oi) 
  {
    this.finder = finder;
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
    var apriltag = finder.getCurrentTagData();

    if (oi.getDriverBButton())
    {
      headlight.setHeadlight(true);
      if (apriltag.isValid())
      {
       //rotation = (0.02 + (drivetrain.getChassisSpeeds().vyMetersPerSecond * 0.01)) * (0 - apriltag.yaw);
       rotation = turnController.calculate(apriltag.yaw,  0);
       SmartDashboard.putNumber("turnController-rotation", rotation);
       MathUtil.clamp(rotation, -1.5, 1.5);
        setRotate(rotation, 1.0);
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
