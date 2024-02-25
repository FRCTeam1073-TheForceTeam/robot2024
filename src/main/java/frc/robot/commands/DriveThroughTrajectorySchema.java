package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  PIDController xController;
  PIDController yController;
  ProfiledPIDController thetaController;
  TrapezoidProfile.Constraints thetaConstraints;
  double currentTime;
  double maxVelocity;
  double maxAngularVelocity;
  double maxAcceleration;
  double alpha;
  double endTime;

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
    xController = new PIDController(
      1.0, 
      0.0, 
      0.02
    );

    yController = new PIDController(
      1.0, 
      0.0, 
      0.02
    );

    thetaConstraints = new TrapezoidProfile.Constraints(3 * maxAngularVelocity, 4 * maxAcceleration);
    thetaController = new ProfiledPIDController(
      1.0, 
      0.0,
      0.1, 
      thetaConstraints
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
    xTrajectory.clear();
    yTrajectory.clear();
    thetaTrajectory.clear();
    for(int i = 0; i < wayPoints.size(); i++)
    {
      xTrajectory.put(trajectoryTime, wayPoints.get(i).getX());
      yTrajectory.put(trajectoryTime, wayPoints.get(i).getY());
      thetaTrajectory.put(trajectoryTime, MathUtils.wrapAngleRadians(wayPoints.get(i).getRotation().getRadians()));
      //update time appropriately
      if(i < wayPoints.size() - 1)
      {
        Transform2d difference = new Transform2d(wayPoints.get(i), wayPoints.get(i + 1));
        double tTime = difference.getTranslation().getNorm() / maxVelocity;
        double rTime = Math.abs(difference.getRotation().getRadians()) / maxAngularVelocity;
        trajectoryTime += Math.max(tTime, rTime);
      }
    }
    endTime = trajectoryTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(Drivetrain drivetrain) 
  {
    currentTime = 0.01;
    startPose = drivetrain.getOdometry();
    posePoints = new ArrayList<Pose2d>();
    posePoints.add(0,startPose);
    posePoints.addAll(posePointInput);
    endPose = posePoints.get(posePoints.size() - 1);
    generateTrajectory(posePoints);
    xController.reset();
    yController.reset();
    thetaController.reset(drivetrain.getHeadingRadians());
    System.out.println("Drive Through Trajectory  " + posePoints.size());
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() 
  {
    robotPose = drivetrain.getOdometry();
    
    //Pose2d state = new Pose2d(-1,0, new Rotation2d(0));
    // Transform2d difference = new Transform2d(robotPose, state);
    // double xVelocity = -alpha * difference.getX();
    // double yVelocity = alpha * difference.getY();
    // double angularVelocity = -0.6 * difference.getRotation().getRadians();

    xController.setSetpoint(xTrajectory.get(currentTime).doubleValue());
    yController.setSetpoint(yTrajectory.get(currentTime).doubleValue());
    thetaController.setGoal(thetaTrajectory.get(currentTime).doubleValue());

    // double xVelocity = xController.calculate(robotPose.getX());
    // double yVelocity = yController.calculate(robotPose.getY());
    // double thetaVelocity  = thetaController.calculate(robotPose.getRotation().getRadians());

    double xError = xTrajectory.get(currentTime).doubleValue() - robotPose.getX();
    double yError = yTrajectory.get(currentTime).doubleValue() - robotPose.getY();
    double thetaError = MathUtils.wrapAngleRadians(thetaTrajectory.get(currentTime).doubleValue() - robotPose.getRotation().getRadians());

    double xVelocity = 0.5 * (Math.exp(1.0 * Math.abs(xError) - 1) * Math.signum(xError));
    double yVelocity = 0.5 * (Math.exp(1.0 * Math.abs(yError) - 1) * Math.signum(yError));
    double thetaVelocity  = 1.0 * (Math.exp(1.0 * Math.abs(thetaError) - 1) * Math.signum(thetaError));

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);

    SmartDashboard.putNumber("Robot pose x", robotPose.getX());
    SmartDashboard.putNumber("Robot Pose y", robotPose.getY());
    // SmartDashboard.putNumber("Difference X", difference.getX());
    // SmartDashboard.putNumber("Difference y", difference.getY());
    SmartDashboard.putNumber("Trajectory Time", currentTime);
    // SmartDashboard.putNumber("state X", goalState.getX());
    // SmartDashboard.putNumber("state Y", goalState.getY());
    // SmartDashboard.putNumber("state Rotation", goalState.getRotation().getRadians());

    // xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    // yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    // angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularVelocity, maxAngularVelocity);

    SmartDashboard.putNumber("Trajectory Speed X", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Speed Y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Trajectory Angular Speed", speeds.omegaRadiansPerSecond);


//    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, 
//      Rotation2d.fromDegrees(drivetrain.getHeading()));

    setTranslate(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 1.0);
    setRotate(speeds.omegaRadiansPerSecond, 1.0);
    
    if(currentTime < endTime)
    {
      currentTime += 0.02;
    }
    else
    {
      currentTime = endTime;
    }
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
    var error = robotPose.minus(endPose);
     
    if ((Math.abs(error.getTranslation().getNorm()) < distanceTolerance || maxVelocity == 0)
      && (Math.abs(error.getRotation().getRadians()) < angleTolerance || maxAngularVelocity == 0)) 
    {
      System.out.println("DriveThroughTrajectory Is Finished");
      return true;
    }
    return false;
  }
}
