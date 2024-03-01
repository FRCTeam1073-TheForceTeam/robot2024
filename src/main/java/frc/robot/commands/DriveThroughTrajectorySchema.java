package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MathUtils;

public class DriveThroughTrajectorySchema extends MotionSchema {
  /** Creates a new DriveThroughTrajectory. */

  double distanceTolerance = 0.1;
  double angleTolerance = 0.1;

  Drivetrain drivetrain;
  ChassisSpeeds speeds;
  Pose2d startPose;
  Pose2d endPose;
  Pose2d robotPose;
  ArrayList<Pose2d> posePoints;
  ArrayList<Pose2d> posePointInput;
  InterpolatingDoubleTreeMap xTrajectory;
  InterpolatingDoubleTreeMap yTrajectory;
  InterpolatingDoubleTreeMap thetaTrajectory;
  InterpolatingDoubleTreeMap xTrajectoryVelocity;
  InterpolatingDoubleTreeMap yTrajectoryVelocity;
  InterpolatingDoubleTreeMap thetaTrajectoryVelocity;
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

  /** Constructs a DriveThroughTrajectory
   * 
   * @param ds variable for driveSubsystem
   * @param start start position
   * @param posePointList list of waypoints the robot should go through
   * @param maxVelocity maximum robot velocity
   * @param maxAngularVelocity maximum angular velocity
   * @param maxAcceleration maximum robot acceleration
   */
  public DriveThroughTrajectorySchema(Drivetrain ds, ArrayList<Pose2d> posePointList, 
    double maxVelocity, double maxAngularVelocity, double maxAcceleration) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    startPose = new Pose2d();
    posePointInput = posePointList;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAcceleration = maxAcceleration;
    xTrajectory = new InterpolatingDoubleTreeMap();
    yTrajectory = new InterpolatingDoubleTreeMap(); 
    thetaTrajectory = new InterpolatingDoubleTreeMap();
    xTrajectoryVelocity = new InterpolatingDoubleTreeMap();
    yTrajectoryVelocity = new InterpolatingDoubleTreeMap();
    thetaTrajectoryVelocity = new InterpolatingDoubleTreeMap();

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

  /** Generates trajectory for robot to go through by creating different trajectory states for each waypoints
   * 
   * @param wayPoints list of waypoints the robot should go through
   * @return generated trajectory for robot to follow
   */
  public void generateTrajectory(ArrayList<Pose2d> wayPoints)
  {
    double trajectoryTime = 0;
    double lastTrajectoryTime = 0;
    xTrajectory.clear();
    yTrajectory.clear();
    thetaTrajectory.clear();
    xTrajectoryVelocity.clear();
    yTrajectoryVelocity.clear();
    thetaTrajectoryVelocity.clear();
    for(int i = 0; i < wayPoints.size(); i++)
    {
      xTrajectory.put(trajectoryTime, wayPoints.get(i).getX());
      yTrajectory.put(trajectoryTime, wayPoints.get(i).getY());
      thetaTrajectory.put(trajectoryTime, MathUtils.wrapAngleRadians(wayPoints.get(i).getRotation().getRadians()));
      if (i == 0)
      {
        xTrajectoryVelocity.put(trajectoryTime, drivetrain.getChassisSpeeds().vxMetersPerSecond);
        yTrajectoryVelocity.put(trajectoryTime, drivetrain.getChassisSpeeds().vyMetersPerSecond);
        thetaTrajectoryVelocity.put(trajectoryTime, drivetrain.getChassisSpeeds().omegaRadiansPerSecond);
      }
      else
      {
        xTrajectoryVelocity.put(trajectoryTime, (wayPoints.get(i).getX() - wayPoints.get(i - 1).getX()) / (trajectoryTime - lastTrajectoryTime));
        yTrajectoryVelocity.put(trajectoryTime, (wayPoints.get(i).getY() - wayPoints.get(i - 1).getY()) / (trajectoryTime - lastTrajectoryTime));
        thetaTrajectoryVelocity.put(trajectoryTime, MathUtils.wrapAngleRadians(
            wayPoints.get(i).getRotation().getRadians() - wayPoints.get(i - 1).getRotation().getRadians()
          )
            / (trajectoryTime - lastTrajectoryTime));
      }
      //update time appropriately
      if(i < wayPoints.size() - 1)
      {
        Transform2d difference = new Transform2d(wayPoints.get(i), wayPoints.get(i + 1));
        double distance = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
        // double tTime = difference.getTranslation().getNorm() / maxVelocity;
        double tTime = distance / maxVelocity;
        double rTime = Math.abs(difference.getRotation().getRadians()) / maxAngularVelocity;
        lastTrajectoryTime = trajectoryTime;
        trajectoryTime += Math.max(tTime, rTime);
      }
    }
    endTime = trajectoryTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) 
  {
    startTime = Timer.getFPGATimestamp();
    currentTime = 0.01;
    startPose = drivetrain.getOdometry();
    posePoints = new ArrayList<Pose2d>();
    posePoints.add(0,startPose);
    posePoints.addAll(posePointInput);
    endPose = posePoints.get(posePoints.size() - 1);
    generateTrajectory(posePoints);
    xController.reset();
    yController.reset();
    thetaController.reset();
    System.out.println("Drive Through Trajectory  " + posePoints.size());
    System.out.println("End time: " + endTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() 
  {
    currentTime = Timer.getFPGATimestamp() - startTime;
    robotPose = drivetrain.getOdometry();

    // double xError = xTrajectory.get(currentTime).doubleValue() - robotPose.getX();
    // double yError = yTrajectory.get(currentTime).doubleValue() - robotPose.getY();
    // double thetaError = MathUtils.wrapAngleRadians(thetaTrajectory.get(currentTime).doubleValue() - robotPose.getRotation().getRadians());

    // double xVelocity = 1.5 * xError;
    // double yVelocity = 1.5 * yError;
    // double thetaVelocity  = 1.0 * thetaError;

    // double xVelocity = xController.calculate(robotPose.getX(), xTrajectory.get(currentTime).doubleValue());
    // double yVelocity = yController.calculate(robotPose.getY(), yTrajectory.get(currentTime).doubleValue());
    // double thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), thetaTrajectory.get(currentTime).doubleValue());
    if (currentTime >= endTime)
    {
      xVelocity = xController.calculate(robotPose.getX(), xTrajectory.get(currentTime).doubleValue());
      yVelocity = yController.calculate(robotPose.getY(), yTrajectory.get(currentTime).doubleValue());
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), thetaTrajectory.get(currentTime).doubleValue());
    }
    else
    {
      xVelocity = xController.calculate(robotPose.getX(), xTrajectory.get(currentTime).doubleValue()) + 0.2 * xTrajectoryVelocity.get(currentTime);
      yVelocity = yController.calculate(robotPose.getY(), yTrajectory.get(currentTime).doubleValue()) + 0.2 * yTrajectoryVelocity.get(currentTime);
      thetaVelocity = thetaController.calculate(robotPose.getRotation().getRadians(), thetaTrajectory.get(currentTime).doubleValue()) + 
        0.2 * thetaTrajectoryVelocity.get(currentTime);
    }
    

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    SmartDashboard.putNumber("Robot pose x", robotPose.getX());
    SmartDashboard.putNumber("Robot Pose y", robotPose.getY());
    SmartDashboard.putNumber("Trajectory Time", currentTime);

    SmartDashboard.putNumber("Trajectory Velocity X", xTrajectoryVelocity.get(currentTime));
    SmartDashboard.putNumber("Trajectory Velocity Y", yTrajectoryVelocity.get(currentTime));
    SmartDashboard.putNumber("Trajectory Velocity Theta", thetaTrajectoryVelocity.get(currentTime));

    // xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    // yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    // angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularVelocity, maxAngularVelocity);

    SmartDashboard.putNumber("Trajectory Speed X", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Speed Y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Angular Speed", speeds.omegaRadiansPerSecond);

    setTranslate(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 1.0);
    setRotate(speeds.omegaRadiansPerSecond, 1.0);
    
    // if(currentTime < endTime)
    // {
    //   currentTime += 0.02;
    // }
    // else
    // {
    //   currentTime = endTime;
    // }
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
  public boolean isFinished() 
  {
    if (currentTime > endTime + 1.0) 
    {
      System.out.println("DriveThroughTrajectory Is Finished");
      return true;
    }
    return false;
  }
}
