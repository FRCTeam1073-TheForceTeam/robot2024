// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends Diagnostics 
{
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;
  private ChassisSpeeds chassisSpeeds;
  private boolean debug = false;
  private SwerveModulePosition[] modulePositions;
  private Pigeon2 pigeon2;
  private double maximumLinearSpeed = 1.0;
  private double[] rates = new double[3];
  private boolean parkingBrakeOn = false;

  /** Creates a new DriveSubsystem. */
  public Drivetrain()
  {
    pigeon2 = new Pigeon2(13);
    var error = pigeon2.getConfigurator().apply(new Pigeon2Configuration());
    if (error != StatusCode.OK) 
    {
      System.out.println(String.format("PIGEON IMU ERROR: %s", error.toString()));
    }
    error = pigeon2.setYaw(0);

    // Make space for four swerve modules:
    modules = new SwerveModule[4];
    modulePositions = new SwerveModulePosition[4];

    //front left
    SwerveModuleIDConfig moduleIDConfig = new SwerveModuleIDConfig(9, 5, 1);
    // moduleIDConfig.driveMotorID = 9; // moduleIDConfig.steerMotorID = 5; // moduleIDConfig.steerEncoderID = 1;

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 0;
    moduleConfig.position = new Translation2d(Preferences.getDouble("Drive.ModulePositions", 0.5017), Preferences.getDouble("Drive.ModulePositions", 0.5017));
    //moduleConfig.steerRotationOffset = Preferences.getDouble("Drive.Module0.SteerRotationsOffset", 0); //0.0579;

    modules[0] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[0] = new SwerveModulePosition();

    //front right
    moduleIDConfig = new SwerveModuleIDConfig(10, 6, 2);
    // moduleIDConfig.driveMotorID = 10; // moduleIDConfig.steerMotorID = 6; // moduleIDConfig.steerEncoderID = 2;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 1;
    moduleConfig.position = new Translation2d(Preferences.getDouble("Drive.ModulePositions", 0.5017), -Preferences.getDouble("Drive.ModulePositions", 0.5017));
    //moduleConfig.steerRotationOffset = Preferences.getDouble("Drive.Module1.SteerRotationOffset", 0); // 0.2141;

    modules[1] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[1] = new SwerveModulePosition();

    //back left
    moduleIDConfig = new SwerveModuleIDConfig(11, 7, 3);
    // moduleIDConfig.driveMotorID = 11; // moduleIDConfig.steerMotorID = 7; // moduleIDConfig.steerEncoderID = 3;

    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 2;
    moduleConfig.position = new Translation2d(-Preferences.getDouble("Drive.ModulePositions", 0.5017), Preferences.getDouble("Drive.ModulePositions", 0.5017));
    //moduleConfig.steerRotationOffset = Preferences.getDouble("Drive.Module2.SteerRotationOffset", 0); // -0.2897

    modules[2] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[2] = new SwerveModulePosition();

    //back right
    moduleIDConfig = new SwerveModuleIDConfig(12, 8, 4);
    // moduleIDConfig.driveMotorID = 12; // moduleIDConfig.steerMotorID = 8; // moduleIDConfig.steerEncoderID = 4;
    moduleConfig = new SwerveModuleConfig(); // Gets preferences and defaults for fields.
    moduleConfig.moduleNumber = 3;
    moduleConfig.position = new Translation2d(-Preferences.getDouble("Drive.ModulePositions", 0.5017), -Preferences.getDouble("Drive.ModulePositions", 0.5017));
    //moduleConfig.steerRotationOffset = Preferences.getDouble("Drive.Module3.SteerRotationOffset", 0); // -0.1635

    modules[3] = new SwerveModule(moduleConfig, moduleIDConfig);
    modulePositions[3] = new SwerveModulePosition();

    // Create our kinematics class
    kinematics = new SwerveDriveKinematics(
      modules[0].position,
      modules[1].position,
      modules[2].position,
      modules[3].position
    );

    // Create odometry:
    modules[0].updatePosition(modulePositions[0]);
    modules[1].updatePosition(modulePositions[1]);
    modules[2].updatePosition(modulePositions[2]);
    modules[3].updatePosition(modulePositions[3]);
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()), modulePositions, new Pose2d(0,0,new Rotation2d(Math.PI)));

    // Configure maximum linear speed for limiting:
    maximumLinearSpeed = Preferences.getDouble("Drive.MaximumLinearSpeed", 3.5);
    SmartDashboard.putNumber("Debug Power", 0.0);
    // Initial chassis speeds are zero:
    chassisSpeeds = new ChassisSpeeds(0,0,0);
  }

  // Initialize preferences for this class:
  public static void initPreferences() 
  {
    // Preferences.initDouble("Drive.Module0.SteerAngleOffset", 2.879); // Radians.
    // Preferences.initDouble("Drive.Module1.SteerAngleOffset", 1.866);
    // Preferences.initDouble("Drive.Module2.SteerAngleOffset", 2.422);
    // Preferences.initDouble("Drive.Module3.SteerAngleOffset", 1.109);
    Preferences.initDouble("Drive.MaximumLinearSpeed", 3.5); // Meters/second
    Preferences.initDouble("Drive.ModulePositions", 0.5017);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("DriveSubsystem");
    // builder.addDoubleProperty("Actual Module 1 speed", modules[1]::getDriveVelocity, null);
    // builder.addDoubleProperty("Actual Module 1 angle", modules[1]::getSteeringAngle, null);

    builder.addDoubleProperty("Odometry x", odometry.getPoseMeters()::getX, null);
    builder.addDoubleProperty("Odometry y", odometry.getPoseMeters()::getY, null);
    builder.addDoubleProperty("Odometry Heading", this::getHeading, null);
    builder.addDoubleProperty("Odometry wrapped Heading", this::getWrappedHeading, null);

    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Roll", this::getRoll, null);
    builder.addDoubleProperty("Pitch Rate", this::getPitchRate, null);
  }

  public String getDiagnostics() 
  {
    String result = modules[0].getDiagnostics();
    result += modules[1].getDiagnostics();
    result += modules[2].getDiagnostics();
    result += modules[3].getDiagnostics();

    //Pigeon2_Faults faults = new Pigeon2_Faults();
    // pigeon2.getFaults(faults);
    // if(faults.hasAnyFault()){
    //   result += faults.toString();
    // }
    StatusCode error = pigeon2.clearStickyFaults(500);
    if (error != StatusCode.OK) {
        result += String.format(" Cannot contact the pigeon.");
    }
    
    //Check errors for all hardware
    return result;
  }

  @Override
  public void runDiagnostics() {
    String result = new String();
    boolean isOK = true;
    //TODO: run diagnostics here
    super.setDiagnosticResult(result);
    super.setOK(isOK);
  }

  public void setDebugMode(boolean debug) 
  {
    this.debug = debug;
  }

  //Returns IMU heading in degrees
  public double getHeading() 
  {
    return -pigeon2.getAngle();
  }

  // Wraps the heading
  public double getWrappedHeading()
  {
    double heading = getHeading() % 360;

    if(heading >= 0)
    {
      return heading;
    }

    else
    {
      return 360 + heading;
    }
  }

  public double getPitch()
  {
    return pigeon2.getPitch().getValue();
  }

  public double getRoll()
  {
    return pigeon2.getRoll().getValue();
  }

  public double getPitchRate()
  {
    return pigeon2.getAccelerationY().getValue();
  }

  // Reset IMU heading to zero degrees
  public void zeroHeading() 
  {
    //TODO:Change value to whatever value you need it to be
    pigeon2.setYaw(0);
  }

  // Set the commanded chassis speeds for the drive subsystem.
  public void setChassisSpeeds(ChassisSpeeds speeds)
  {
    SmartDashboard.putNumber("ChassisSpeed x", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeed rotation", speeds.omegaRadiansPerSecond);
    chassisSpeeds = speeds;
  }

  // Return the measured chassis speeds for the drive subsystem.
  public ChassisSpeeds getChassisSpeeds()
  {
    SwerveModuleState[] wheelStates = new SwerveModuleState[4];
    wheelStates[0] = new SwerveModuleState();
    wheelStates[1] = new SwerveModuleState();
    wheelStates[2] = new SwerveModuleState();
    wheelStates[3] = new SwerveModuleState();

    wheelStates[0].speedMetersPerSecond = modules[0].getDriveVelocity();
    wheelStates[1].speedMetersPerSecond = modules[1].getDriveVelocity();
    wheelStates[2].speedMetersPerSecond = modules[2].getDriveVelocity();
    wheelStates[3].speedMetersPerSecond = modules[3].getDriveVelocity();

    wheelStates[0].angle = Rotation2d.fromRotations(modules[0].getSteerRotations());
    wheelStates[1].angle = Rotation2d.fromRotations(modules[1].getSteerRotations());
    wheelStates[2].angle = Rotation2d.fromRotations(modules[2].getSteerRotations());
    wheelStates[3].angle = Rotation2d.fromRotations(modules[3].getSteerRotations());

    return kinematics.toChassisSpeeds(wheelStates);
  }

  // puts the motors in brake mode
  public void setBrakes(boolean brakeOn)
  {
    modules[0].setBrake(brakeOn);
    modules[1].setBrake(brakeOn);
    modules[2].setBrake(brakeOn);
    modules[3].setBrake(brakeOn);
  }

  // returns the wheel positions
  public SwerveDriveKinematics getKinematics()
  {
    return kinematics;
  }

  public void updateOdometry() 
  {
    modules[0].updatePosition(modulePositions[0]);
    modules[1].updatePosition(modulePositions[1]);
    modules[2].updatePosition(modulePositions[2]);
    modules[3].updatePosition(modulePositions[3]);
    odometry.update(Rotation2d.fromDegrees(getHeading()), modulePositions);
  }

  public void resetOdometry(Pose2d where)
  {
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), modulePositions, where);
  }

  public Pose2d getOdometry()
  {
    return new Pose2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY(), Rotation2d.fromDegrees(getHeading()));
  }

  public Pose3d get3dOdometry()
  {
    // return odometry position as a pose 3d
    Pose2d odo = getOdometry();
    //TODO: use internal roll and pitch methods later
    return new Pose3d(odo.getX(), odo.getY(), 0.0, new Rotation3d(getRoll(), getPitch(), getHeading()));
  }
  
  @Override
  public void periodic()
  {

    if (!debug && !parkingBrakeOn) //disables motors when parking brakes are active
    {
      // This method will be called once per scheduler run
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, maximumLinearSpeed);

      SmartDashboard.putNumber("Module 0 unoptimized", states[0].angle.getRotations()); 
      SmartDashboard.putNumber("Module 1 unoptimized", states[1].angle.getRotations()); 
      SmartDashboard.putNumber("Module 2 unoptimized", states[2].angle.getRotations()); 
      SmartDashboard.putNumber("Module 3 unoptimized", states[3].angle.getRotations()); 

      states[0] = SwerveModuleState.optimize(states[0], Rotation2d.fromRotations(modules[0].getSteerRotations()));
      states[1] = SwerveModuleState.optimize(states[1], Rotation2d.fromRotations(modules[1].getSteerRotations()));
      states[2] = SwerveModuleState.optimize(states[2], Rotation2d.fromRotations(modules[2].getSteerRotations()));
      states[3] = SwerveModuleState.optimize(states[3], Rotation2d.fromRotations(modules[3].getSteerRotations()));

      // SmartDashboard.putNumber("Module 0 optimized", states[0].angle.getRadians());
      // SmartDashboard.putNumber("Module 1 optimized", states[1].angle.getRadians());
      // SmartDashboard.putNumber("Module 2 optimized", states[2].angle.getRadians());
      // SmartDashboard.putNumber("Module 3 optimized", states[3].angle.getRadians());

      modules[0].setCommand(states[0].angle.getRotations(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRotations(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRotations(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRotations(), states[3].speedMetersPerSecond);

      // modules[0].setCommand(states[0].angle.getRotations(), states[0].speedMetersPerSecond);
      // modules[1].setCommand(states[1].angle.getRotations(), states[1].speedMetersPerSecond);
      // modules[2].setCommand(states[2].angle.getRotations(), states[2].speedMetersPerSecond);
      // modules[3].setCommand(states[3].angle.getRotations(), states[3].speedMetersPerSecond);


      // SmartDashboard.putNumber("Actual Module 1 speed", modules[1].getDriveVelocity());
      // SmartDashboard.putNumber("Actual Module 1 angle", modules[1].getSteeringAngle());
      // SmartDashboard.putNumber("Commanded Speed", states[1].speedMetersPerSecond);
      // SmartDashboard.putNumber("Commanded angle", states[1].angle.getRadians());
    }
    if(debug)
    { //in debug mode
      SmartDashboard.putNumber("Module 0 Velocity in Rotations per Second", modules[0].getDriveRawVelocity());

      setDebugDrivePower(SmartDashboard.getNumber("Debug Power", 0.0));
    }
    updateOdometry();
    // SmartDashboard.putNumber("Odometry.X", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Odometry.Y", odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("Odometry.Heading", this.getHeading());
    // SmartDashboard.putNumber("Odometry.Wrapped.Heading", this.getWrappedHeading());

    // SmartDashboard.putNumber("Pitch", getPitch());
    // SmartDashboard.putNumber("Roll", getRoll());
    // SmartDashboard.putNumber("Rate", getPitchRate());
  }


  // rotates all the wheels to be facing inwards and stops the motors to hold position
  public void parkingBrake(boolean parkingBrakeOn) 
  {
    this.parkingBrakeOn = parkingBrakeOn;
    if (parkingBrakeOn)
    {
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      states[0] = optimize(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), Rotation2d.fromRotations(modules[0].getSteerRotations()));
      states[1] = optimize(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), Rotation2d.fromRotations(modules[1].getSteerRotations()));
      states[2] = optimize(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), Rotation2d.fromRotations(modules[2].getSteerRotations()));
      states[3] = optimize(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), Rotation2d.fromRotations(modules[3].getSteerRotations()));

      modules[0].setCommand(states[0].angle.getRotations(), states[0].speedMetersPerSecond);
      modules[1].setCommand(states[1].angle.getRotations(), states[1].speedMetersPerSecond);
      modules[2].setCommand(states[2].angle.getRotations(), states[2].speedMetersPerSecond);
      modules[3].setCommand(states[3].angle.getRotations(), states[3].speedMetersPerSecond);
    }
  }

  public boolean getParkingBrake()
  {
    return parkingBrakeOn;
  }

  public void setDebugSpeed(double speed) // sets the speed directly
  {
    modules[0].setDriveVelocity(speed);
    modules[1].setDriveVelocity(speed);
    modules[2].setDriveVelocity(speed);
    modules[3].setDriveVelocity(speed);
  }

  public void setDebugAngle(double power) // sets the angle directly
  {
    //SmartDashboard.putNumber("Debug Angle", angle);

    modules[0].setDebugRotate(power);
    modules[1].setDebugRotate(power);
    modules[2].setDebugRotate(power);
    modules[3].setDebugRotate(power);
  }

  public void setDebugDrivePower(double power) // sets the power directly
  {
    modules[0].setDebugTranslate(power);
    modules[1].setDebugTranslate(power);
    modules[2].setDebugTranslate(power);
    modules[3].setDebugTranslate(power);
  }


  // optimizes the module states to take the shortest path to the desired position
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle)
  {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90)
    {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  

  // places the desired angle to be in a 0 to 360 scope to minimize distance for wheels to rotate 
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) 
  {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0)
    {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } 
    else
    {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound)
    {
      newAngle += 360;
    }
    while (newAngle > upperBound)
    {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180)
    {
      newAngle -= 360;
    } 
    else if (newAngle - scopeReference < -180) 
    {
      newAngle += 360;
    }
    return newAngle;
  }
}
