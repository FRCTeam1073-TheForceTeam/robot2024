// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CollectorArm extends DiagnosticsSubsystem {
  
  public static enum POSE{
    START,
    STOW_INTERMEDIATE,
    STOW_INTERMEDIATE_2,
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

  // Rate Limiters For Each Axis
  // private final SlewRateLimiter liftLimiter;
  // private final SlewRateLimiter extendLimiter;

  // Command For Each Axis
  public MotionMagicVoltage liftPositionVoltage;
  public MotionMagicVoltage extendPositionVoltage;

  // Map for profiled motion
  private InterpolatingDoubleTreeMap armMap;
  
  // CANBus for this subsystem
  private static final String kCANbus = "CANivore";

  private StatusCode configError_Extend;
  private StatusCode configError_Lift;

  //TODO: Change all of these values
  private final double liftLength = 0;
  private final double armLength = 0;
  // Approximage masses of arm segments for gravity compensation:
  private final double liftArmMass = 0; 
  private final double extendArmMass = 0; 
  private final double gravityCompensationGain = 1.25;            // Increase this to increase the overall amount of gravity compensation.
  private final double gravityCompensationLiftGain = 0.0125; // 1/80 gear ratio
  private final double gravityCompensationExtendGain = 0.025;     // 1/40 gear ratio

  // Axis scaling and offset:
  private final double liftAbsoluteOffset = 0;
  private final double extendAbsoluteOffset = 0;
  private final double liftGearRatio = 25.0;
  private final double liftDriveRatio = 4.0;
  private final double extendGearRatio = 25.0;
  private final double extendPulleyRadius = 0.01375;
  private final double liftRadiansPerRotation = (2*Math.PI)/(liftDriveRatio*liftGearRatio);
  private final double extendMetersPerRotation = (2*Math.PI*extendPulleyRadius)/extendGearRatio;


  // TODO: update values in radians
  private final double minLiftAngle = 0;
  private final double maxLiftAngle = 2.09649; 

  // TODO: update values (in meters)
  private final double minExtend = -0.08;
  private final double maxExtend = 0.108;

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
  private double lift_kI = 7.5;
  private double lift_kD = 1.5;
  private double lift_kF = 0;
  private double lift_kG = 0.9;

  // PID gains for extension controller.
  private double extend_kP = 145;
  private double extend_kI = 4;
  private double extend_kD = 11;
  private double extend_kS = 0.5;

  private POSE currentPose;

  private boolean extendInterpolateFlag = true;

  /** Creates a new CollectorArm. */
  public CollectorArm() {
    liftMotor = new TalonFX(13, kCANbus); 
    extendMotor = new TalonFX(15, kCANbus);
    liftMotorFault = new MotorFault(liftMotor, 13);
    extendMotorFault = new MotorFault(extendMotor, 15);

    // Rate limiter between target value and commanded value for smooth motion.
    // liftLimiter = new SlewRateLimiter(0.5); 
    // extendLimiter = new SlewRateLimiter(1); 

    // Position-based command.
    liftPositionVoltage = new MotionMagicVoltage(0).withSlot(0);
    extendPositionVoltage = new MotionMagicVoltage(0).withSlot(0);

    currentPose = POSE.START;

    configureHardware();

    armMap = new InterpolatingDoubleTreeMap();
    //setUpInterpolator();
  }

  @Override
  public void periodic() 
  {
    updateFeedback();
    // if(extendInterpolateFlag){
    //   targetExtendLength = interpolateExtendPosition(currentLiftAngle);
    // }
    commandedExtendLength = limitExtendLength(targetExtendLength);
    commandedLiftAngle = limitLiftAngle(targetLiftAngle);
    runLiftMotor(commandedLiftAngle);
    runExtendMotor(commandedExtendLength);
    updateDiagnostics();
  }


  private void updateFeedback() {
    // Sensor angles should be divided by the appropriate ticks per radian
    //currentLiftAngle = liftMotor.getPosition().refresh().getValue() * liftRadiansPerRotation - liftAbsoluteOffset;
    currentLiftAngle = liftMotor.getPosition().refresh().getValue();
    SmartDashboard.putNumber("lift motor position rotations", liftMotor.getPosition().refresh().getValue());
    currentLiftVelocity = liftMotor.getVelocity().refresh().getValueAsDouble() * liftRadiansPerRotation;
    //currentExtendLength = extendMotor.getPosition().refresh().getValue() * extendMetersPerRotation - extendAbsoluteOffset;
    currentExtendLength = extendMotor.getPosition().refresh().getValue();
    SmartDashboard.putNumber("Extend motor position rotations", extendMotor.getPosition().refresh().getValue());
    currentExtendVelocity = extendMotor.getVelocity().refresh().getValueAsDouble() * extendMetersPerRotation;
  }

  public double limitLiftAngle(double liftAngle) {
    return MathUtil.clamp(liftAngle, minLiftAngle, maxLiftAngle);
  }

  public double limitExtendLength(double extendLength) {
    return MathUtil.clamp(extendLength, minExtend, maxExtend);
  }

  public double getCurrentLiftAngle() {
    return currentLiftAngle;
  }

  public double getCurrentExtendLength() {
    return currentExtendLength;
  }

  public void setTargetLiftAngle(double target) {
    targetLiftAngle = limitLiftAngle(target);
  }

  public void setTargetExtendLength(double target) {
    targetExtendLength = limitExtendLength(target);
  }

  public void setPoseName(POSE pose) {
    currentPose = pose;
  }

  // Returns target lift angle in radians
  public double getTargetLiftAngle() {
    return targetLiftAngle;
  }

  // Returns target extend length in meters
  public double getTargetExtendLength() {
    return targetExtendLength;
  }

  // Returns currently commanded lift angle (limited, rate limited) in radians.
  public double getCommandedLiftAngle() {
    return commandedLiftAngle;
  }

  // Returns currenyl commanded extend length (limited, rate limited) in meters.
  public double getCommandedExtendLength() {
    return commandedExtendLength;
  }

   public double getCurrentLiftVelocity() {
    return currentLiftVelocity;
  }

  public double getCurrentExtendVelocity() {
    return currentExtendVelocity;
  }

  public POSE getPoseName() {
    return currentPose;
  }

  public void setExtendInterpolateFlag(boolean flagValue){
    extendInterpolateFlag = flagValue;
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

  /** Takes a lift angle and calculates the target extend length */
  public double interpolateExtendPosition(double currentliftAngle){
    return armMap.get(currentLiftAngle);
  }


  private void runLiftMotor(double liftAngle)
  {
    // conversions: Position to drive toward in rotations = revolutations = 2pi
    //double liftAngleRotations = (liftAngle + liftAbsoluteOffset) / liftRadiansPerRotation;
    double liftAngleRotations = liftAngle;
    double liftFeedForward = lift_kG * Math.cos(currentLiftAngle);
    SmartDashboard.putNumber("Commanded Motor LiftAngleRotations", liftAngleRotations);
    liftMotor.setControl(liftPositionVoltage.withPosition(liftAngleRotations).withFeedForward(liftFeedForward));
  }

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
    // // liftConfigs.Feedback.RotorToSensorRatio = (150.0 / 7.0);
     liftConfigs.Feedback.SensorToMechanismRatio = 1 / liftRadiansPerRotation;  // This should be used for remote CANCoder with continuous wrap.
    // liftConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // liftConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    liftConfigs.MotionMagic.MotionMagicCruiseVelocity = 3; 
    liftConfigs.MotionMagic.MotionMagicAcceleration = 4; 
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
    //extendConfigs.Feedback.RotorToSensorRatio = (150.0 / 7.0);
    extendConfigs.Feedback.SensorToMechanismRatio = 1 / extendMetersPerRotation;  // This should be used for remote CANCoder with continuous wrap.
    //extendConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //extendConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    extendConfigs.MotionMagic.MotionMagicCruiseVelocity = 1;
    extendConfigs.MotionMagic.MotionMagicAcceleration = 0.7;
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

    var error1 = liftMotor.getConfigurator().apply(liftMotorClosedLoopConfig, 0.5);
    if(!error1.isOK()){
      System.err.print(String.format("LIFT MOTOR ERROR: %s", error1.toString()));
      setDiagnosticsFeedback(error1.getDescription(), false);
    }
    
    var error2 = extendMotor.getConfigurator().apply(extendMotorClosedLoopConfig, 0.5);
    if(!error2.isOK()){
      System.err.print(String.format("EXTEND MOTOR ERROR: %s", error2.toString()));
      setDiagnosticsFeedback(error2.getDescription(), false);
    } 
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
    
    //builder.addBooleanProperty("ok", this::isOK, null);
    //builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
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
