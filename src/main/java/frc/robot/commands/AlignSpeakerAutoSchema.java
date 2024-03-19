// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class AlignSpeakerAutoSchema extends MotionSchema implements Activate
{
  AprilTagFinder finder;
  double rotation;
  boolean active;

  /** Creates a new AlignToSpeakerSchema. */
  public AlignSpeakerAutoSchema(AprilTagFinder finder) 
  {
    this.finder = finder;
    active = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) {}

  @Override
  public void activate(boolean active)
  {
    this.active = active;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void update(Drivetrain drivetrain) 
  {
    var apriltag = finder.getCurrentTagData();

    if (apriltag.isValid() && active)
    {
      rotation = 0.01 * (160 - apriltag.cx);
      MathUtil.clamp(rotation, -0.7, 0.7);

      setRotate(rotation, 3.0);
    }
    else //if((Math.abs(160 - apriltag.cx) < 20) && !finder.tagFound())
    {
      setRotate(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
   return false;
  }
}
