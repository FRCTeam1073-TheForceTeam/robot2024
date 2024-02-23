// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopDrive extends SchemaArbiter 
{
  double angleTolerance = 0.05;
  double startAngle;
  double desiredAngle;
  ChassisSpeeds chassisSpeeds;
  Pose2d targetRotation;
  Pose2d robotRotation;
  Drivetrain m_drivetrain;
  OI m_OI;
  private boolean fieldCentric;
  private boolean parked = false;
  ChassisSpeeds speeds;
  double last_error = 0; //for snap-to-positions derivative
  double last_time = 0; //for snap-to-positions derivative
  boolean lastParkingBreakButton = false;
  boolean lastRobotCentricButton = false;
  TeleopTranslateSchema translateSchema;
  TeleopRotateSchema rotateSchema;
  TeleopHeadingHoldSchema holdSchema;
  PointAtTargetSchema targetSchema;
  boolean pointAtTarget;

  PIDController snapPidProfile;

  // Teleop drive velocity scaling:
  private final static double maximumLinearVelocity = 3.5;   // Meters/second
  private final static double maximumRotationVelocity = 4.0; // Radians/second


  /** Creates a new Teleop. */
  public TeleopDrive(Drivetrain ds, OI oi) 
  {
    super(ds, true, false);
    super.setName("Teleop Drive");
    m_drivetrain = ds;
    m_OI = oi;
    fieldCentric = true;
    startAngle = ds.getHeadingDegrees();
    desiredAngle = startAngle;
    pointAtTarget = false;
    snapPidProfile = new PIDController(
      0.05, 
      0.0, 
      0.0);
    translateSchema = new TeleopTranslateSchema(m_OI, maximumLinearVelocity);
    rotateSchema = new TeleopRotateSchema(m_OI, maximumRotationVelocity);
    holdSchema = new TeleopHeadingHoldSchema(m_OI);
    targetSchema = new PointAtTargetSchema(maximumRotationVelocity, new Pose2d(2.5, 0.0, new Rotation2d()), 1.75);
    addSchema(translateSchema);
    addSchema(rotateSchema);
    addSchema(holdSchema);
    addSchema(targetSchema);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ds);
  }

  public void initPreferences()
  {
    Preferences.initDouble("Snap to Position P", 0.1);
    Preferences.initDouble("Snap to Position I", 0);
    Preferences.initDouble("Snap to Position D", 0);
    Preferences.initDouble("Snap to Position Max Acceleration", 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("TeleopDrive: Init");
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    //multiples the angle by a number from 1 to the square root of 30:
    // double mult1 = 1.0 + (m_OI.getDriverLeftTrigger() * ((Math.sqrt(25)) - 1));
    // double mult2 = 1.0 + (m_OI.getDriverRightTrigger() * ((Math.sqrt(25)) - 1));

    // double leftY = m_OI.getDriverLeftY();
    // double leftX = m_OI.getDriverLeftX();
    // double rightX = m_OI.getDriverRightX();
    //sets deadzones on the controller to extend to .05:
    // if(Math.abs(leftY) < .15) {leftY = 0;}
    // if(Math.abs(leftX) < .15) {leftX = 0;}
    // if(Math.abs(rightX) < .15) {rightX = 0;}

    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(leftY * 0.5, leftX * 0.5, rightX); //debug
    if (m_OI.getYButtonDriver())
    {
      pointAtTarget = !pointAtTarget;
    }
    if (pointAtTarget)
    {
      rotateSchema.setActive(false);
      targetSchema.setActive(true);
    }
    else
    {
      rotateSchema.setActive(true);
      targetSchema.setActive(false);
    }
    SmartDashboard.putBoolean("Rotate Schema Active", rotateSchema.getActive());
    SmartDashboard.putBoolean("Target Schema Active", targetSchema.getActive());

    if (m_OI.getFieldCentricToggle() && lastRobotCentricButton == false)
    {
      fieldCentric = !fieldCentric;
    }
    lastRobotCentricButton = m_OI.getFieldCentricToggle();
    SmartDashboard.putBoolean("Field Centric", fieldCentric);
    SmartDashboard.putBoolean("Parking Brake", parked);

    if(m_OI.getLeftBumper() && lastParkingBreakButton == false)
    {
      parked = !parked;
    }
    lastParkingBreakButton = m_OI.getLeftBumper();
    if(parked && !m_drivetrain.getParkingBrake())
    {
      m_drivetrain.parkingBrake(true);
    }
    if(!parked && m_drivetrain.getParkingBrake())
    {
      m_drivetrain.parkingBrake(false);
    }
    else // if (fieldCentric)
    {

      // double vx = MathUtil.clamp(-(leftY * maximumLinearVelocity / 25 )* mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
      // double vy = MathUtil.clamp(-(leftX * maximumLinearVelocity / 25 ) * mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity);
      // double w = MathUtil.clamp(-(rightX * maximumRotationVelocity / 25) * mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);

      // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      //   vx,
      //   vy,
      //   w,
      //   Rotation2d.fromDegrees(m_drivetrain.getHeading())); // get fused heading
      // m_drivetrain.setChassisSpeeds(speeds);
      
      
      translateSchema.update(m_drivetrain);
      rotateSchema.update(m_drivetrain);
      holdSchema.update(m_drivetrain);
      targetSchema.update(m_drivetrain);
    }
    // else
    // {
    //   Robot centric driving.
    //   speeds = new ChassisSpeeds();
    //   speeds.vxMetersPerSecond = MathUtil.clamp(-(leftY * maximumLinearVelocity / 25 )* mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity); 
    //   speeds.vyMetersPerSecond = MathUtil.clamp(-(leftX * maximumLinearVelocity / 25)* mult1 * mult2, -maximumLinearVelocity, maximumLinearVelocity); 
    //   speeds.omegaRadiansPerSecond = MathUtil.clamp(-(rightX * maximumRotationVelocity / 25)* mult1 * mult2, -maximumRotationVelocity, maximumRotationVelocity);
    //   m_drivetrain.setChassisSpeeds(speeds); 
    // }
    
    // Allow driver to zero the drive subsystem heading for field-centric control.
    if(m_OI.getMenuButton()){
      m_drivetrain.zeroHeading();
    }

    if(m_OI.getAButton()){
      Rotation2d zeroRotate = new Rotation2d();
      Pose2d zero = new Pose2d(0.0, 0.0, zeroRotate);
      m_drivetrain.resetOdometry(zero);
    }


    SmartDashboard.putBoolean("Field Centric ", fieldCentric);

    super.execute();

  }

  // public double snapToHeading(double currentAngle, double targetAngle, double joystickDesired){
  //   if(targetAngle == 361){
  //     targetAngle = joystickDesired;
  //   }
  //   double error = currentAngle - targetAngle;
  //   while(error < -180){error += 360;}
  //   while(error > 180){error -= 360;}
  //   SmartDashboard.putNumber("Angle Error", error);
    
  //   error = error * Math.PI / 180;
  //   double new_error = error;
  //   double current_time = System.currentTimeMillis();
  //   double derivative = (new_error - last_error)/(current_time - last_time);
  //   last_time = current_time;
  //   last_error = error;
  //   error = MathUtil.clamp(error, -maximumRotationVelocity, maximumRotationVelocity);

  //   return MathUtil.clamp(error * .7 + derivative * .1, -maximumRotationVelocity, maximumRotationVelocity) / maximumRotationVelocity;
    
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    if (interrupted) {
      System.out.println("TeleopDrive: Interrupted!");
    }
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
