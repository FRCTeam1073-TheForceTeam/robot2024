// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CollectorArm extends DiagnosticsSubsystem {
  
  enum POSE{
    START,
    STOW,
    COLLECT,
    SCORE
  }

  TalonFX liftMotor;
  TalonFX extendMotor;
  
  MotorFault liftMotorFault = new MotorFault(liftMotor, 15);
  MotorFault extendMotorFault = new MotorFault(extendMotor, 16);
  private double liftSpeed;
  private double extendSpeed; 

  InterpolatingTreeMap<Double, Double> armMap;
  
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
  private final double liftTicksPerRadian = -20.656*4*2048/(2*Math.PI);
  private final double extendTicksPerRadian = 20.656*4*2048/(2*Math.PI);

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
  
  // fill in PID values
  private double lift_kP = 0;
  private double lift_kI = 0;
  private double lift_kD = 0;
  private double lift_kF = 0;

  private double extend_kP = 0;
  private double extend_kI = 0;
  private double extend_kD = 0;
  private double extend_kF = 0;

  /** Creates a new Arm. */
  public CollectorArm() {
    liftMotor = new TalonFX(15); // TODO: set device id
    extendMotor = new TalonFX(16); // TODO: set device id
    liftSpeed = 0;
    extendSpeed = 0;
    setUpMotors();
  }

  @Override
  public void periodic() 
  {
    runExtendMotor(extendSpeed);
    runLiftMotor(liftSpeed);
    updateCurrentPositions();
    
    // interpolator tree map member smth
  }


  private void updateCurrentPositions() {
    // Sensor angles should be divided by the appropriate ticks per radian
    currentLiftAngle = liftMotor.getPosition().getValue(); //Todo: conversions
    currentExtendLength = extendMotor.getPosition().getValue();
  }

  public double getCurrentLiftAngle() {
    return currentLiftAngle;
  }

  public double getCurrentExtendLength() {
    return currentExtendLength;
  }

  public void setTargetLiftAngle(double target) {
    targetLiftAngle = target;
  }

  public double getTargetLiftAngle() {
    return targetLiftAngle;
  }

  public double getTargetExtendLength() {
    return targetExtendLength;
  }

  /** Takes a lift angle and calculates the target extend length */
  public void interpolate(double liftAngle){

    targetExtendLength = 0.0; //TODO: Implement
    // maybe make a interpolate that takes the extend length and gets the radians it needs to rotate to get that desired length
  }

  public void runLiftMotor(double liftSpeed)
  {
    liftMotor.setControl(new PositionVoltage(liftSpeed * liftTicksPerRadian)); //TODO: Position to drive toward in rotations
    // liftMotor.setControl(new VelocityVoltage(liftSpeed * liftTicksPerRadian));
  }

  public void runExtendMotor(double extendSpeed)
  {
    extendMotor.setControl(new PositionVoltage(extendSpeed * extendTicksPerRadian)); 
    // extendMotor.setControl(new VelocityVoltage(extendSpeed * extendTicksPerRadian));
  }

  public void setLiftSpeed(double liftSpeed)
  {
    this.liftSpeed = liftSpeed;
  }

  public double getLiftSpeed()
  {
    return liftSpeed;
  }

  public void setExtendSpeed(double extendSpeed)
  {
    this.extendSpeed = extendSpeed;
  }

  public double getExtendSpeed()
  {
    return extendSpeed;
  }

  public void setUpMotors() {
    //PID loop setting for lift motor
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

    extendMotor.getConfigurator().apply(extendMotorClosedLoopConfig);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Lift Speed", this::getLiftSpeed, this::setLiftSpeed);
    builder.addDoubleProperty("Extend Speed", this::getExtendSpeed, this::setExtendSpeed);
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
