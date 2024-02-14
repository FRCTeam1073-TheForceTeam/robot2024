package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveThroughTrajectorySchema extends MotionSchema {
  /** Creates a new DriveThroughTrajectory. */

  double distanceTolerance = 0.2;
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
  double currentTime;
  double maxVelocity;
  double maxAngularVelocity;
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
   * @param alpha speed multiplier
   */
  public DriveThroughTrajectorySchema(Drivetrain ds, ArrayList<Pose2d> posePointList, 
    double maxVelocity, double maxAngularVelocity, double maxAcceleration, double alpha) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = ds;
    startPose = new Pose2d();
    posePointInput = posePointList;
    this.maxVelocity = maxVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
    this.alpha = alpha;
    xTrajectory = new InterpolatingDoubleTreeMap();
    yTrajectory = new InterpolatingDoubleTreeMap(); // TODO: fix this
    thetaTrajectory = new InterpolatingDoubleTreeMap();
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
      thetaTrajectory.put(trajectoryTime, wayPoints.get(i).getRotation().getRadians());
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
    System.out.println("Drive Through Trajectory  " + posePoints.size());
  }

  // Called every time the scheduler runs while the command is scheduled.
  //interpolates the trajectory to get the desired pose at a given time and sets speed proportional to the difference
  @Override
  public void execute() 
  {
    robotPose = drivetrain.getOdometry();
    Pose2d state = new Pose2d(
      new Translation2d(xTrajectory.get(currentTime).doubleValue(), 
      yTrajectory.get(currentTime).doubleValue()),
      Rotation2d.fromRadians(thetaTrajectory.get(currentTime).doubleValue()));
    Transform2d difference = new Transform2d(robotPose, state);
    double xVelocity = -alpha * difference.getX();
    double yVelocity = alpha * difference.getY();
    double angularVelocity = -0.6 * difference.getRotation().getRadians();


    SmartDashboard.putNumber("Robot pose x", robotPose.getX());
    SmartDashboard.putNumber("Robot Pose y", robotPose.getY());
    SmartDashboard.putNumber("Difference X", difference.getX());
    SmartDashboard.putNumber("Difference y", difference.getY());
    SmartDashboard.putNumber("Trajectory Time", currentTime);
    SmartDashboard.putNumber("state X", state.getX());
    SmartDashboard.putNumber("state Y", state.getY());
    SmartDashboard.putNumber("state Rotation", state.getRotation().getRadians());

    xVelocity = MathUtil.clamp(xVelocity, -maxVelocity, maxVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxVelocity, maxVelocity);
    angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularVelocity, maxAngularVelocity);

    SmartDashboard.putNumber("Trajectory Speed X", xVelocity);
    SmartDashboard.putNumber("Trajectory Speed Y", yVelocity);
    SmartDashboard.putNumber("Trajectory Angular Speed", angularVelocity);


//    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, 
//      Rotation2d.fromDegrees(drivetrain.getHeading()));

    setTranslate(xVelocity, yVelocity, 1.0);
    setRotate(angularVelocity, 1.0);
    
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
     
    if (error.getTranslation().getNorm() < distanceTolerance && Math.abs(error.getRotation().getRadians()) < angleTolerance) 
    {
      System.out.println("DriveThroughTrajectory Is Finished");
      return true;
    }
    return false;
  }
}
