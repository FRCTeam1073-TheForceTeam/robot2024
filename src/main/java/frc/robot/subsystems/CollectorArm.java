// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CollectorArm extends DiagnosticsSubsystem {
  
  public static enum POSE{
    START,
    STOW_INTERMEDIATE,
    STOW,
    HANDOFF,
    AMP
  }

  // Motors:
  private TalonFX liftMotor;
  private TalonFX extendMotor;
  
  // Motor Fault Trackers:
  MotorFault liftMotorFault;
  MotorFault extendMotorFault;

  // Command For Each Axis
  public MotionMagicVoltage liftPositionVoltage;
  public MotionMagicVoltage extendPositionVoltage;

  // Map for profiled motion
  //private InterpolatingDoubleTreeMap armMap;
  
  // CANBus for this subsystem
  private static final String kCANbus = "CANivore";

  private StatusCode configError_Extend;
  private StatusCode configError_Lift;

  // Axis scaling and offset:
  private final double liftGearRatio = 25.0;
  private final double liftDriveRatio = 4.0;
  private final double extendGearRatio = 25.0;
  private final double extendPulleyRadius = 0.01375;
  private final double liftRadiansPerRotation = (2*Math.PI)/(liftDriveRatio*liftGearRatio);
  private final double extendMetersPerRotation = (2*Math.PI*extendPulleyRadius)/extendGearRatio;

  private final double minLiftAngleRad = 0;
  private final double maxLiftAngleRad = 2.09649; 

  private final double minExtendMeters = -0.08;
  private final double maxExtendMeters = 0.107;

  // Internal lift state variables:
  private double currentLiftAngle = 0;
  private double currentLiftVelocity = 0;
  private double commandedLiftAngle = 0;
  private double targetLiftAngle = 0;

  // Internal extension state variables:
  private double currentExtendLength = 0;
  private double currentExtendVelocity = 0;
  private double commandedExtendLength = 0;
  private double targetExtendLength = 0;
  
  // PID gains for lift controller.
  private double lift_kP = 16;
  private double lift_kI = 6;
  private double lift_kD = 1.5;
  private double lift_kF = 0;
  private double lift_kG = 0.8;

  // PID gains for extension controller.
  private double extend_kP = 145;
  private double extend_kI = 4;
  private double extend_kD = 11;
  private double extend_kS = 0.5;

  private POSE currentPose;

  /** Creates a new CollectorArm. */
  public CollectorArm() {
    liftMotor = new TalonFX(13, kCANbus); 
    extendMotor = new TalonFX(15, kCANbus);
    liftMotorFault = new MotorFault(liftMotor, 13);
    extendMotorFault = new MotorFault(extendMotor, 15);

    // Position-based command.
    liftPositionVoltage = new MotionMagicVoltage(0).withSlot(0);
    extendPositionVoltage = new MotionMagicVoltage(0).withSlot(0);

    currentPose = POSE.START;

    configureHardware();

    // armMap = new InterpolatingDoubleTreeMap();
    // setUpInterpolator();
  }

  @Override
  public void periodic() 
  {
    updateFeedback();

    commandedExtendLength = limitExtendLength(targetExtendLength);
    commandedLiftAngle = limitLiftAngle(targetLiftAngle);
    runLiftMotor(commandedLiftAngle);
    runExtendMotor(commandedExtendLength);
    updateDiagnostics();
  }

  private void updateFeedback() {
    currentLiftAngle = liftMotor.getPosition().refresh().getValue();
    currentExtendLength = extendMotor.getPosition().refresh().getValue();
    currentLiftVelocity = liftMotor.getVelocity().refresh().getValueAsDouble() * liftRadiansPerRotation;
    currentExtendVelocity = extendMotor.getVelocity().refresh().getValueAsDouble() * extendMetersPerRotation;
  }

  /* Limits lift angle in radians and axtend length meters */
  public double limitLiftAngle(double liftAngle) {
    return MathUtil.clamp(liftAngle, minLiftAngleRad, maxLiftAngleRad);
  }
  public double limitExtendLength(double extendLength) {
    return MathUtil.clamp(extendLength, minExtendMeters, maxExtendMeters);
  }

  /* Gets current lift angle in radians and extend length in meters */
  public double getCurrentLiftAngle() {
    return currentLiftAngle;
  }
  public double getCurrentExtendLength() {
    return currentExtendLength;
  }

  /* Sets target lift angle in radians and extend length in meters */
  public void setTargetLiftAngle(double target) {
    targetLiftAngle = limitLiftAngle(target);
  }
  public void setTargetExtendLength(double target) {
    targetExtendLength = limitExtendLength(target);
  }

  /* Returns target lift angle in radians and extend length in meters */
  public double getTargetLiftAngle() {
    return targetLiftAngle;
  }
  public double getTargetExtendLength() {
    return targetExtendLength;
  }

  /* Returns current commanded lift angle and extend length (limited with clamp) in radians and meters, respectively. */
  public double getCommandedLiftAngle() {
    return commandedLiftAngle;
  }
  public double getCommandedExtendLength() {
    return commandedExtendLength;
  }

  /* Returns actual lift velocity in radians/sec and actual extend velocity in meters/sec */
  public double getCurrentLiftVelocity() {
    return currentLiftVelocity;
  }
  public double getCurrentExtendVelocity() {
    return currentExtendVelocity;
  }

  /* Gets and sets POSE name */
  public void setPoseName(POSE pose) {
    currentPose = pose;
  }
  public POSE getPoseName() {
    return currentPose;
  }

  // public void setUpInterpolator() {
  //   armMap.clear();
    
  //   // Keys are lift angles, values are extension distances.
  //   // Fill out the rest of the table (angle, extendLength)

  //   armMap.put(0.0, 0.0); 
  //   armMap.put(0.076416015625, 0.0);
  //   armMap.put(0.122802734375, 0.0);
  //   armMap.put(0.162109375, 0.0); //STOW 
  //   armMap.put(0.2, 0.04);
  //   armMap.put(0.21, 0.107177734375); 
  //   armMap.put(0.2880859375, 0.107177734375);
  //   armMap.put(0.400390625, -0.041005859375);
  //   armMap.put(0.76171875, -0.0425390625);
  //   armMap.put(1.313720703125, 0.030517578125);
  //   armMap.put(1.9453125, 0.0966796875);
  //   armMap.put(2.70654296875, 0.10986328125);

  //   /*
  //   Adding more points at a steeper slope near important positions will yield a
  //   faster acceleration(limited by motion magic configs) to the target.
    
  //   Right now it is tuned to be relatively fast
  //   */
  // }

  // /** Takes a lift angle and calculates the target extend length */
  // public double interpolateExtendPosition(double currentliftAngle){
  //   return armMap.get(currentLiftAngle);
  // }

  /* Runs lift motor at angle input in radians */
  private void runLiftMotor(double liftAngle)
  {
    // conversions: Position to drive toward in rotations = revolutations = 2pi
    //double liftAngleRotations = (liftAngle + liftAbsoluteOffset) / liftRadiansPerRotation;
    double liftAngleRotations = liftAngle;
    double liftFeedForward = lift_kG * Math.cos(currentLiftAngle);
    SmartDashboard.putNumber("Commanded Motor LiftAngleRotations", liftAngleRotations);
    liftMotor.setControl(liftPositionVoltage.withPosition(liftAngleRotations).withFeedForward(liftFeedForward));
  }

  /* runs extend motor at length input in meters */
  private void runExtendMotor(double extendLength)
  {
    //double extendLengthRotations = (extendLength + extendAbsoluteOffset) / extendMetersPerRotation;
    double extendLengthRotations = extendLength;
    SmartDashboard.putNumber("Commanded Motor ExtendLengthRotations", extendLengthRotations);
    extendMotor.setControl(extendPositionVoltage.withPosition(extendLengthRotations)); 
  }

  private void configureHardware(){

    TalonFXConfiguration extendConfigs = new TalonFXConfiguration();
    TalonFXConfiguration liftConfigs = new TalonFXConfiguration();

    configError_Lift = liftMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    configError_Extend = extendMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    
    liftConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    liftConfigs.Feedback.SensorToMechanismRatio = 1 / liftRadiansPerRotation;

    liftConfigs.MotionMagic.MotionMagicCruiseVelocity = 4; 
    liftConfigs.MotionMagic.MotionMagicAcceleration = 3; 
    liftConfigs.MotionMagic.MotionMagicJerk = 0;

    liftMotor.getConfigurator().apply(liftConfigs);
    liftMotor.setPosition(0);

    liftMotor.setNeutralMode(NeutralModeValue.Brake);

    // PID loop setting for lift motor
    var liftMotorClosedLoopConfig = new Slot0Configs();
    liftMotorClosedLoopConfig.withKP(lift_kP);
    liftMotorClosedLoopConfig.withKI(lift_kI);
    liftMotorClosedLoopConfig.withKD(lift_kD);
    liftMotorClosedLoopConfig.withKV(lift_kF);

    extendConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    extendConfigs.Feedback.SensorToMechanismRatio = 1 / extendMetersPerRotation;

    extendConfigs.MotionMagic.MotionMagicCruiseVelocity = 1;
    extendConfigs.MotionMagic.MotionMagicAcceleration = 0.5;
    extendConfigs.MotionMagic.MotionMagicJerk = 0;

    extendMotor.getConfigurator().apply(extendConfigs);
    extendMotor.setPosition(0);
    
    extendMotor.setNeutralMode(NeutralModeValue.Coast);

    //PID loop setting for extend motor
    var extendMotorClosedLoopConfig = new Slot0Configs();
    extendMotorClosedLoopConfig.withKP(extend_kP);
    extendMotorClosedLoopConfig.withKI(extend_kI);
    extendMotorClosedLoopConfig.withKD(extend_kD);
    extendMotorClosedLoopConfig.withKS(extend_kS);

    configError_Lift = liftMotor.getConfigurator().apply(liftMotorClosedLoopConfig, 0.5);
    configError_Extend = extendMotor.getConfigurator().apply(extendMotorClosedLoopConfig, 0.5);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector Arm");
    builder.addDoubleProperty("Lift Angle", this::getCurrentLiftAngle, null);
    builder.addDoubleProperty("Extend Length", this::getCurrentExtendLength, null);
    builder.addDoubleProperty("Target Lift Angle", this::getTargetLiftAngle, null);
    builder.addDoubleProperty("Target Extend Length", this::getTargetExtendLength, null);
    builder.addDoubleProperty("Commanded Lift Angle", this::getCommandedLiftAngle, null);
    builder.addDoubleProperty("Commanded Extend Length", this::getCommandedExtendLength, null);
    builder.addDoubleProperty("Current Lift Velocity", this::getCurrentLiftVelocity, null);
    builder.addDoubleProperty("Current Extend Velocity", this::getCurrentExtendVelocity, null);
  }

  @Override
  public boolean updateDiagnostics() 
  {
    String result = "";
    boolean OK = true;
    
    if(liftMotorFault.hasFaults() || extendMotorFault.hasFaults())
    {
        OK = false;
    }

    if (!configError_Lift.isOK()) 
    {
        result += configError_Lift.getDescription();
        OK = false;
    }

    if (!configError_Extend.isOK()) 
    {
        result += configError_Extend.getDescription();
        OK = false;
    }
    
    result += liftMotorFault.getFaults() + extendMotorFault.getFaults();
    return setDiagnosticsFeedback(result, OK);
  }
}
