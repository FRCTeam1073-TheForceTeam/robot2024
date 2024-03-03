package frc.robot.commands;

import java.util.ArrayList;

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
    }
    currentTime = Timer.getFPGATimestamp() - startTime;
    robotPose = drivetrain.getOdometry();
    
    // Compute position and velocity desired from where we actually are:
    PathFeedback pathFeedback = path.getPathFeedback(currentSegmentIndex, robotPose);

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
    

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    SmartDashboard.putNumber("Robot pose x", robotPose.getX());
    SmartDashboard.putNumber("Robot Pose y", robotPose.getY());
    SmartDashboard.putNumber("Trajectory Time", currentTime);

    SmartDashboard.putNumber("Trajectory Velocity X", pathFeedback.velocity.get(0,0));
    SmartDashboard.putNumber("Trajectory Velocity Y", pathFeedback.velocity.get(1,0));

    // xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    // yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    // angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularVelocity, maxAngularVelocity);

    SmartDashboard.putNumber("Trajectory Speed X", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Speed Y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Angular Speed", speeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Segment Index", currentSegmentIndex);

    setTranslate(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 1.0);
    setRotate(speeds.omegaRadiansPerSecond, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees(drivetrain.getHeadingDegrees()));
    drivetrain.setTargetChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  if (currentSegmentIndex < 0) return true; // Finished if we don't have a good index.

  Path.Segment seg = path.segments.get(currentSegmentIndex);
  if (path.atEndPoint(currentSegmentIndex, drivetrain.getOdometry())) 
  {

    // Cancel comamnd for this segment:
    if (seg.entryCommand != null) 
    {
      CommandScheduler.getInstance().cancel(seg.entryCommand);
    }
    // Kick off our exit command:
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

