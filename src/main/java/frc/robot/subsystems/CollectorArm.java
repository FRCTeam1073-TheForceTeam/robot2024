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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CollectorArm extends DiagnosticsSubsystem {
  
  enum POSE{
    START,
    STOW,
    COLLECT,
    SCORE
  }

  private TalonFX liftMotor;
  private TalonFX extendMotor;
  
  MotorFault liftMotorFault;
  MotorFault extendMotorFault;

  private TalonFXConfiguration liftMotorConfigurator = new TalonFXConfiguration();
  private TalonFXConfiguration extendMotorConfigurator = new TalonFXConfiguration();
  private final SlewRateLimiter liftLimiter;
  private final SlewRateLimiter extendLimiter;

  public PositionVoltage liftPositionVoltage;
  public PositionVoltage extendPositionVoltage;
  public VelocityVoltage liftVelocityVoltage;
  public VelocityVoltage extendVelocityVoltage;

  InterpolatingTreeMap<Double, Double> armMap;
  
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

  private final double liftAbsoluteOffset = 0;
  private final double extendAbsoluteOffset = 0;
  private final double liftGearRatio = 25.0;
  private final double liftDriveRatio = 4.0;
  private final double extendGearRatio = 25.0;
  private final double extendPulleyRadius = 0.01375;
  private final double liftRadiansPerRotation = (2*Math.PI)/(liftDriveRatio*liftGearRatio);
  private final double extendMetersPerRotation = (2*Math.PI*extendPulleyRadius)/extendGearRatio;
  


  // TODO: fill out values in radians
  private final double minLiftAngle = 0;
  private final double maxLiftAngle = 0; 

  // TODO: fill out values (unit tbd)
  private final double minExtend = 0;
  private final double maxExtend = 0;

  private double currentLiftAngle;
  private double currentExtendLength;
  private double targetLiftAngle;
  private double targetExtendLength;
  private double currentLiftVelocity;
  private double commandedLiftAngle;
  private double commandedExtendLength;
  private double currentExtendVelocity;
  private double targetLiftVelocity;
  private double targetExtendVelocity;

  
  // fill in PID values
  private double lift_kP = 0.3;
  private double lift_kI = 0;
  private double lift_kD = 0;
  private double lift_kF = 0;

  private double extend_kP = 0.3;
  private double extend_kI = 0;
  private double extend_kD = 0;
  private double extend_kF = 0;

  /** Creates a new Arm. */
  public CollectorArm() {
    liftMotor = new TalonFX(15, kCANbus); // TODO: set device id
    extendMotor = new TalonFX(16,kCANbus); // TODO: set device id
    liftMotorFault = new MotorFault(liftMotor, 15);
    extendMotorFault = new MotorFault(extendMotor, 16);
    liftLimiter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    extendLimiter = new SlewRateLimiter(0.5); 
    liftVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    extendVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    liftPositionVoltage = new PositionVoltage(0).withSlot(0);
    extendPositionVoltage = new PositionVoltage(0).withSlot(0);
    configureHardware();
    setUpInterpolator();
  }

  @Override
  public void periodic() 
  {
    updateCurrentPositions();
    // targetLiftAngle = m_OI.getOperatorRightY();
    // targetExtendLength = m_OI.getOperatorLeftY();
    commandedExtendLength = extendLimiter.calculate(limitExtendLength(targetExtendLength));
    commandedLiftAngle = liftLimiter.calculate(limitLiftAngle(targetLiftAngle));
    targetExtendLength = interpolateExtendPosition(currentLiftAngle);
  }



  public void updateCurrentPositions() {
    // Sensor angles should be divided by the appropriate ticks per radian
    currentLiftAngle = liftMotor.getPosition().getValue() * liftRadiansPerRotation - liftAbsoluteOffset; //Todo: conversions
    currentExtendLength = extendMotor.getPosition().getValue() * extendMetersPerRotation - extendAbsoluteOffset;
  }

  public double limitLiftAngle(double liftAngle) {
    return MathUtil.clamp(liftAngle, minLiftAngle, maxLiftAngle);
  }

  public double limitExtendLength(double extendLength) {
    return MathUtil.clamp(extendLength, minExtend, minExtend);
  }

  public double getCurrentLiftAngle() {
    return currentLiftAngle;
  }

  public double getCurrentExtendLength() {
    return currentExtendLength;
  }

  public void setTargetLiftAngle(double target) {
    targetLiftAngle = liftLimiter.calculate(limitLiftAngle(target));
  }

  public void setTargetExtendLength(double target) {
    targetExtendLength = liftLimiter.calculate(limitExtendLength(target));
  }

  public double getTargetLiftAngle() {
    return targetLiftAngle;
  }

  public double getTargetExtendLength() {
    return targetExtendLength;
  }

  public double getCommandedLiftAngle() {
    return commandedLiftAngle;
  }

  public double getCommandedExtendLength() {
    return commandedExtendLength;
  }

   public double getCurrentLiftVelocity() {
    return currentLiftVelocity;
  }

  public double getCurrentExtendVelocity() {
    return currentExtendVelocity;
  }

  public double getTargetLiftVelocity() {
    return targetLiftVelocity;
  }

  public double getTargetExtendVelocity() {
    return targetExtendVelocity;
  }


  public void setUpInterpolator() {
    //armMap = new InterpolatingTreeMap<Double, Double>();
    //armMap.put(Double.valueOf(0.0), Double.valueOf(2.0)); 
    //TODO: fill out the rest of the table (angle, extendLength)
  }

  /** Takes a lift angle and calculates the target extend length */
  public double interpolateExtendPosition(double currentliftAngle){
    //return armMap.get(currentLiftAngle);
    return 0;
  }


  public void runLiftMotor(double liftAngle)
  {
    double liftAngleRotations = (liftAngle + liftAbsoluteOffset) / liftRadiansPerRotation;
    liftMotor.setControl(liftPositionVoltage.withPosition(liftAngleRotations)); //TODO: conversions: Position to drive toward in rotations = revolutations = 2pi
  }

  public void runExtendMotor(double extendLength)
  {
    double extendLengthRotations = (extendLength + extendAbsoluteOffset) / extendMetersPerRotation;
    extendMotor.setControl(extendPositionVoltage.withPosition(extendLengthRotations)); 
  }

  public void runLiftMotorAtVelocity(double liftVelocity)
  {
    // liftMotor.setControl(new VelocityVoltage(Velocity * liftTicksPerMeter));
  }

  public void runExtendMotorAtVelocity(double extendVelocity)
  {
    // extendMotor.setControl(new VelocityVoltage(Velocity * extendTicksPerMeter));
  }

  public void configureHardware(){
    //PID loop setting for lift motor
    liftMotor.setPosition(0);
    extendMotor.setPosition(0);
    var liftMotorClosedLoopConfig = new Slot0Configs();

    liftMotorClosedLoopConfig.withKP(lift_kP);
    liftMotorClosedLoopConfig.withKI(lift_kI);
    liftMotorClosedLoopConfig.withKD(lift_kD);
    liftMotorClosedLoopConfig.withKV(lift_kF);

    liftMotor.getConfigurator().apply(liftMotorClosedLoopConfig);

    //PID loop setting for collect motor
    var extendMotorClosedLoopConfig = new Slot0Configs();

    extendMotorClosedLoopConfig.withKP(extend_kP);
    extendMotorClosedLoopConfig.withKI(extend_kI);
    extendMotorClosedLoopConfig.withKD(extend_kD);
    extendMotorClosedLoopConfig.withKV(extend_kF);

    var error1 = liftMotor.getConfigurator().apply(liftMotorClosedLoopConfig, 0.5);
    if(!error1.isOK()){
      System.err.print(String.format("Module %d LIFT MOTOR ERROR: %s", error1.toString()));
      setDiagnosticsFeedback(error1.getDescription(), false);
    }
    var error2 = extendMotor.getConfigurator().apply(extendMotorClosedLoopConfig, 0.5);
    if(!error2.isOK()){
      System.err.print(String.format("Module %d EXTEND MOTOR ERROR: %s", error2.toString()));
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
    // builder.addDoubleProperty("Target Lift Velocity", this::getTargetLiftVelocity, null);
    // builder.addDoubleProperty("Target Extend Velocity", this::getTargetExtendVelocity, null);
    

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
