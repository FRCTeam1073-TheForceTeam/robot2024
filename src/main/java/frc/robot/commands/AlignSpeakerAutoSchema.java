// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class AlignSpeakerAutoSchema extends MotionSchema implements Activate
{
  AprilTagFinder finder;
  double rotation = 0.0;
  double error = 0.0;
  double last_error = 0.0;
  boolean active = false;
  double center_point = 160.0;

  /** Creates a new AlignToSpeakerSchema. */
  public AlignSpeakerAutoSchema(AprilTagFinder finder) 
  {
    this.finder = finder;
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

    if (apriltag.isValid() && active)
    {
      last_error = error;
      error = (center_point - apriltag.cx);

      // PD controller form:
      rotation = 0.02 * error +  0.0 * (error - last_error) ;
      MathUtil.clamp(rotation, -1.0, 1.0);


      setRotate(rotation, 3.0);
    }
    else
    {
      setRotate(0.0, 0.0);
    }

    SmartDashboard.putBoolean("Align To Speaker Active", apriltag.isValid() && active);
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
