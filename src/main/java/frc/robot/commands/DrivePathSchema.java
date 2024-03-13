package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.Path.PathFeedback;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MathUtils;

public class DrivePathSchema extends MotionSchema {
  /** Creates a new DriveThroughTrajectory. */

  double distanceTolerance = 0.1;
  double angleTolerance = 0.1;

  Drivetrain drivetrain;
  Pose2d robotPose;
  ChassisSpeeds speeds;
  Path path;
  int currentSegmentIndex = -1;

  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double currentTime;
  double maxVelocity;
  double maxAngularVelocity;
  double maxAcceleration;
  double endTime;
  double xVelocity;
  double yVelocity;
  double thetaVelocity;
  double startTime;

 /**
  * Constructs a schema to drive along a given path.
  * @param ds
  * @param path
  */
  public DrivePathSchema(Drivetrain ds, Path path) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    this.path = path;

    xController = new PIDController(
      1.0, 
      0.05, 
      0.01
    );

    yController = new PIDController(
      1.0, 
      0.05, 
      0.01
    );

    thetaController = new PIDController(
      1.2, 
      0.05,
      0.01
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) 
  {
    startTime = Timer.getFPGATimestamp();
    currentTime = 0.01;
    currentSegmentIndex = path.closestSegment(drivetrain.getOdometry());    

    if (currentSegmentIndex != -1 && path.segments.get(currentSegmentIndex).entryCommand != null) {
      CommandScheduler.getInstance().schedule(path.segments.get(currentSegmentIndex).entryCommand);
    }

    xController.reset();
    yController.reset();
    thetaController.reset();
    System.out.println("Drive Path Starting Segment  " + currentSegmentIndex);
    System.out.println("End time: " + endTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() 
  {
    if (currentSegmentIndex < 0) 
    {
      System.out.println("DrivePathSchema: Invalid segment index.");
      // Stop:
      setTranslate(0,0,1);
      setRotate(0,1);
      return; // Don't run.
    }

    

    currentTime = Timer.getFPGATimestamp() - startTime;
    robotPose = drivetrain.getOdometry();
    
    // Compute position and velocity desired from where we actually are:
    PathFeedback pathFeedback = path.getPathFeedback(currentSegmentIndex, robotPose);

    maxVelocity = pathFeedback.velocity.norm();
    maxAngularVelocity = pathFeedback.velocity.norm();
    

    if (currentSegmentIndex >= path.segments.size() - 1)
    {
      // Last point is meant to be a bit different:
      xVelocity = xController.calculate(robotPose.getX(), pathFeedback.pose.getX());
      yVelocity = yController.calculate(robotPose.getY(), pathFeedback.pose.getY());
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), path.finalOrientation);
    }
    else
    {
      xVelocity = xController.calculate(robotPose.getX(), pathFeedback.pose.getX()) + 0.3 * pathFeedback.velocity.get(0,0);
      yVelocity = yController.calculate(robotPose.getY(), pathFeedback.pose.getY()) + 0.3 * pathFeedback.velocity.get(1,0);
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), path.getPathOrientation(currentSegmentIndex, robotPose)); 
    }
    
    // Clamp to maximums for safety:
    xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    thetaVelocity = MathUtil.clamp(thetaVelocity, -maxAngularVelocity, maxAngularVelocity);

    // Create (field-centric) chassis speeds:
    ChassisSpeeds fcSpeeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    SmartDashboard.putNumber("Robot pose x", robotPose.getX());
    SmartDashboard.putNumber("Robot Pose y", robotPose.getY());
    // SmartDashboard.putNumber("Trajectory Time", currentTime);

    SmartDashboard.putNumber("Traj FB Velocity X", pathFeedback.velocity.get(0,0));
    SmartDashboard.putNumber("Traj FB Velocity Y", pathFeedback.velocity.get(1,0));

    SmartDashboard.putNumber("Trajectory Speed X", fcSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Speed Y", fcSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Angular Speed", fcSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Segment Index", currentSegmentIndex);

    // Controlled drive command with weights from our path segment feedback, set our two channels of schema output/w weights.
    setTranslate(fcSpeeds.vxMetersPerSecond, fcSpeeds.vyMetersPerSecond, pathFeedback.translation_weight);
    setRotate(fcSpeeds.omegaRadiansPerSecond, pathFeedback.orientation_weight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Set our schema output to full stop.
    setTranslate(0,0,1);
    setRotate(0,1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  if (currentSegmentIndex < 0) return true; // Finished if we don't have a good index.

  // Otherwise check our segment, and manage command launch as we move along segments.
  Path.Segment seg = path.segments.get(currentSegmentIndex);
  if (path.atEndPoint(currentSegmentIndex, drivetrain.getOdometry())) 
  {

    // Cancel entry comamnd for this segment:
    if (seg.entryCommand != null) 
    {
      CommandScheduler.getInstance().cancel(seg.entryCommand);
    }
    // Kick off our exit command for this segment:
    if (seg.exitCommand != null) 
    {
      CommandScheduler.getInstance().schedule(seg.exitCommand);
    }

    // Move to next path segment:
    currentSegmentIndex = currentSegmentIndex + 1;
    if (currentSegmentIndex >= path.segments.size()) 
    {
      System.out.println("DrivePathSchema: Finished.");
      return true;
    } 
    else 
    {
      // Move to new segment:
      seg = path.segments.get(currentSegmentIndex);
      if (seg.entryCommand != null) 
      {
        CommandScheduler.getInstance().schedule(seg.entryCommand);
      }
    }
  }
  return false; // Keep on going.
}

}

