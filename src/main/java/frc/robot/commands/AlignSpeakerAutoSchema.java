// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Headlight;

public class AlignSpeakerAutoSchema extends MotionSchema implements Activate
{
  AprilTagFinder finder;
  Headlight headlight;
  double rotation = 0.0;
  boolean active = false;
  PIDController turnController = new PIDController(0.15, 0, 0.015);

  /** Creates a new AlignToSpeakerSchema. */
  public AlignSpeakerAutoSchema(AprilTagFinder finder, Headlight headlight) 
  {
    this.finder = finder;
    this.headlight = headlight;
    active = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) {
  }

  @Override
  public void activate(boolean active)
  {
    this.active = active;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    var apriltag = finder.getCurrentTagData();

    if (active)
    {
      headlight.setHeadlight(true);
      if (apriltag.isValid())
      {
        rotation = turnController.calculate(apriltag.yaw,  0);
        SmartDashboard.putNumber("turnController-rotation", rotation);
        MathUtil.clamp(rotation, -1.5, 1.5);
        setRotate(rotation, 1.0);
      }
      else
      {
        setRotate(0.0, 0.0);
      }
    }
    else
    {
      setRotate(0.0, 0.0);
      headlight.setHeadlight(false);
    }

    SmartDashboard.putBoolean("Align To Speaker Active", apriltag.isValid() && active);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    headlight.setHeadlight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   return false;
  }
}
