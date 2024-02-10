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
  
  MotorFault liftMotorFault = new MotorFault(liftMotor, 15);
  MotorFault extendMotorFault = new MotorFault(extendMotor, 16);
  private TalonFXConfiguration liftMotorConfigurator = new TalonFXConfiguration();
  private TalonFXConfiguration extendMotorConfigurator = new TalonFXConfiguration();
  private final SlewRateLimiter liftFilter;
  private final SlewRateLimiter extendFilter;

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
    liftFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    extendFilter = new SlewRateLimiter(0.5); 
    configureHardware();
    setUpInterpolator();
  }

  @Override
  public void periodic() 
  {
    updateCurrentPositions();
    runLiftMotor(targetLiftAngle);
    targetExtendLength = interpolateExtendPosition(currentLiftAngle);
    runExtendMotor(targetExtendLength); 
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

  public void setUpInterpolator() {
    armMap.put(Double.valueOf(0.0), Double.valueOf(2.0)); 
    //TODO: fill out the rest of the table (angle, extendLength)
  }

  /** Takes a lift angle and calculates the target extend length */
  public double interpolateExtendPosition(double currentliftAngle){

    return armMap.get(currentLiftAngle);
  }


  public void runLiftMotor(double liftAngle)
  {
    //liftMotor.setControl(new PositionVoltage(liftFilter.calculate(liftAngle))); //TODO: conversions: Position to drive toward in rotations = revolutations = 2pi

  }

  public void runExtendMotor(double extendLength)
  {
    // extendMotor.setControl(new PositionVoltage(extendFilter.calculate(extendLength))); 
  }

  public void configureHardware(){
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

    var error = liftMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    if(!error.isOK()){
      System.err.print(String.format("Module %d LIFT MOTOR ERROR: %s", error.toString()));
      setDiagnosticsFeedback(error.getDescription(), false);
    }
    error = extendMotor.getConfigurator().apply(new TalonFXConfiguration(), 0.5);
    if(!error.isOK()){
      System.err.print(String.format("Module %d EXTEND MOTOR ERROR: %s", error.toString()));
      setDiagnosticsFeedback(error.getDescription(), false);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Lift Angle", this::getCurrentLiftAngle, null);
    builder.addDoubleProperty("Extend Length", this::getCurrentExtendLength, null);
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
