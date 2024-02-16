// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CollectorArm extends DiagnosticsSubsystem {
  
  enum POSE{
    START,
    STOW,
    COLLECT,
    SCORE
  }

  // Motors:
  private TalonFX liftMotor;
  private TalonFX extendMotor;
  
  // Motor Fault Trackers:
  MotorFault liftMotorFault;
  MotorFault extendMotorFault;

  // Rate Limiters For Each Axis
  private final SlewRateLimiter liftLimiter;
  private final SlewRateLimiter extendLimiter;

  // Command For Each Axis
  public PositionVoltage liftPositionVoltage;
  public PositionVoltage extendPositionVoltage;

  // Map for profiled motion
  private InterpolatingDoubleTreeMap armMap;
  
  // CANBus for this subsystem
  private static final String kCANbus = "CANivore";

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
  private final double liftRadiansPerRotation = (-2*Math.PI)/(liftDriveRatio*liftGearRatio);
  private final double extendMetersPerRotation = (-2*Math.PI*extendPulleyRadius)/extendGearRatio;


  // TODO: update values in radians
  private final double minLiftAngle = -2.09649;
  private final double maxLiftAngle = 0; 

  // TODO: update values (in meters)
  private final double minExtend = -0.1;
  private final double maxExtend = 0.08;

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
  private double lift_kP = 0.7;
  private double lift_kI = 0;
  private double lift_kD = 0;
  private double lift_kF = 0;

  // PID gains for extension controller.
  private double extend_kP = 0.3;
  private double extend_kI = 0;
  private double extend_kD = 0;
  private double extend_kF = 0;

  /** Creates a new CollectorArm. */
  public CollectorArm() {
    liftMotor = new TalonFX(15, kCANbus); 
    extendMotor = new TalonFX(16, kCANbus);
    liftMotorFault = new MotorFault(liftMotor, 15);
    extendMotorFault = new MotorFault(extendMotor, 16);

    // Rate limiter between target value and commanded value for smooth motion.
    liftLimiter = new SlewRateLimiter(0.01); 
    extendLimiter = new SlewRateLimiter(0.01); 

    // Position-based command.
    liftPositionVoltage = new PositionVoltage(0).withSlot(0);
    extendPositionVoltage = new PositionVoltage(0).withSlot(0);

    configureHardware();
    setUpInterpolator();
  }

  @Override
  public void periodic() 
  {
    updateFeedback();
    commandedExtendLength = extendLimiter.calculate(limitExtendLength(targetExtendLength));
    commandedLiftAngle = liftLimiter.calculate(limitLiftAngle(targetLiftAngle));
    runLiftMotor(commandedLiftAngle);
    runExtendMotor(commandedExtendLength);
    //targetExtendLength = interpolateExtendPosition(currentLiftAngle);
  }


  private void updateFeedback() {
    // Sensor angles should be divided by the appropriate ticks per radian
    currentLiftAngle = liftMotor.getPosition().refresh().getValue() * liftRadiansPerRotation - liftAbsoluteOffset;
    currentLiftVelocity = liftMotor.getVelocity().refresh().getValueAsDouble() * liftRadiansPerRotation;
    currentExtendLength = extendMotor.getPosition().refresh().getValue() * extendMetersPerRotation - extendAbsoluteOffset;
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


  public void setUpInterpolator() {
    armMap = new InterpolatingDoubleTreeMap();

    // Keys are lift angles, values are extension distances.
    armMap.put(Double.valueOf(0.0), Double.valueOf(0.0)); 
    armMap.put(Double.valueOf(-0.2), Double.valueOf(0.0));
    armMap.put(Double.valueOf(-0.5), Double.valueOf(0.05));

    //TODO: fill out the rest of the table (angle, extendLength)
  }

  /** Takes a lift angle and calculates the target extend length */
  public double interpolateExtendPosition(double currentliftAngle){
    return armMap.get(currentLiftAngle);
  }


  private void runLiftMotor(double liftAngle)
  {
    // conversions: Position to drive toward in rotations = revolutations = 2pi
    double liftAngleRotations = (liftAngle + liftAbsoluteOffset) / liftRadiansPerRotation;
    liftMotor.setControl(liftPositionVoltage.withPosition(liftAngleRotations)); 
  }

  private void runExtendMotor(double extendLength)
  {
    double extendLengthRotations = (extendLength + extendAbsoluteOffset) / extendMetersPerRotation;
    extendMotor.setControl(extendPositionVoltage.withPosition(extendLengthRotations)); 
  }

 

  private void configureHardware(){

    var error = liftMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    if (!error.isOK()) 
    {
        System.err.print(String.format("Lift Motor ERROR: %s", error.toString()));
        setDiagnosticsFeedback(error.getDescription(), false);
    }

    error = extendMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    if (!error.isOK()) 
    {
        System.err.println(String.format("Extend MOTOR ERROR: %s", error.toString()));
        setDiagnosticsFeedback(error.getDescription(), false);
    }

    // Zero motor positions at start: Robot must be in hard-stop pose.
    liftMotor.setPosition(0);
    extendMotor.setPosition(0);

    liftMotor.setNeutralMode(NeutralModeValue.Brake);
    extendMotor.setNeutralMode(NeutralModeValue.Coast);

    // PID loop setting for lift motor
    var liftMotorClosedLoopConfig = new Slot0Configs();
    liftMotorClosedLoopConfig.withKP(lift_kP);
    liftMotorClosedLoopConfig.withKI(lift_kI);
    liftMotorClosedLoopConfig.withKD(lift_kD);
    liftMotorClosedLoopConfig.withKV(lift_kF);


    //PID loop setting for extend motor
    var extendMotorClosedLoopConfig = new Slot0Configs();
    extendMotorClosedLoopConfig.withKP(extend_kP);
    extendMotorClosedLoopConfig.withKI(extend_kI);
    extendMotorClosedLoopConfig.withKD(extend_kD);
    extendMotorClosedLoopConfig.withKV(extend_kF);

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
    extendMotor.initSendable(builder);
    liftMotor.initSendable(builder);
  }

  @Override
  public boolean updateDiagnostics() 
  {
    String result = "";
    boolean OK = true;

    if(liftMotorFault.hasFaults() || extendMotorFault.hasFaults()){
        OK = false;
    }
    
    result += liftMotorFault.getFaults() + extendMotorFault.getFaults();
    return setDiagnosticsFeedback(result, OK);
  }
}
