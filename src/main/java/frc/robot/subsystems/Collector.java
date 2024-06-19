// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Collector extends DiagnosticsSubsystem {

  private final String kCANbus = "CANivore";
  private TalonFX collectMotor = new TalonFX(14, kCANbus); // same thing
  private MotorFault collectMotorFault = new MotorFault(collectMotor, 14);
  private TalonFXConfiguration collectMotorConfigurator = new TalonFXConfiguration();
  private StatusCode configError;
  private double targetCollectorVelocity = 0; //TODO: find appropriate speed for collector
  private double currentCollectorVelocity = 0;
  private double commandedCollectorVelocity = 0;
  public MotionMagicVelocityVoltage collectorVelocityVoltage;

  private DigitalInput tof1;
  private DutyCycle tof1DutyCycleInput;
  private double tof1DutyCycle;
  private double tof1Freq;
  private double tof1Range;
  private final double tof1CollectorScaleFactor = 3000000/4; // for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million

  private DigitalInput tof2;
  private DutyCycle tof2DutyCycleInput;
  private double tof2DutyCycle;
  private double tof2Freq;
  private double tof2Range;
  private final double tof2CollectorScaleFactor = 3000000/4; // for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million

  private final double collectorGearRatio = 3.0/1.0;
  private final double collectorWheelRadius = 0.0254; //meters
  private final double collectorMeterPerRotations = -collectorWheelRadius * 2 * Math.PI * collectorGearRatio;

  // private final SlewRateLimiter collectorLimiter;

  private final double intakeTicksPerRadian = 2048.0 * collectorGearRatio / (2.0 * Math.PI);
  
  //fill in PID values
  
  private double collect_kP = 0.27;
  private double collect_kI = 0;
  private double collect_kD = 0.001;
  private double collect_kF = 0.15;

  /** Creates a new Collector. */
  public Collector() {

    tof1 = new DigitalInput(0); 
    tof1DutyCycleInput = new DutyCycle(tof1);
    tof1Freq = 0;
    tof1Range = 0;

    tof2 = new DigitalInput(3); 
    tof2DutyCycleInput = new DutyCycle(tof2);
    tof2Freq = 0;
    tof2Range = 0;

    collectorVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0); 
    //collectorLimiter = new SlewRateLimiter(24);

    configureHardware();
  }

  @Override
  public void periodic() 
  {
    //commandedCollectorVelocity = collectorLimiter.calculate(targetCollectorVelocity);
    currentCollectorVelocity = collectMotor.getVelocity().getValueAsDouble() * collectorMeterPerRotations; //meters per second 
    runCollectMotor(targetCollectorVelocity);
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1CollectorScaleFactor * (tof1DutyCycle / tof1Freq - 0.001) / 1000; //supposedly in meters

    tof2Freq = tof2DutyCycleInput.getFrequency();
    tof2DutyCycle = tof2DutyCycleInput.getOutput();
    tof2Range = tof2CollectorScaleFactor * (tof2DutyCycle / tof2Freq - 0.001) / 1000; //supposedly in meters
    updateDiagnostics();
  }
  
  private void runCollectMotor(double vel)
  {
    collectMotor.setControl(collectorVelocityVoltage.withVelocity(vel / collectorMeterPerRotations)); //meters per 
    //collectMotor.setControl(collectorVelocityVoltage.withVelocity(vel)); //meters per 
  }

  public void setTargetCollectorVelocity(double velocity){
    targetCollectorVelocity = velocity;
  }

  public double getTargetCollectorVelocity()
  {
    return targetCollectorVelocity;
  }

  public double getCommandedCollectorVelocity(){
    return commandedCollectorVelocity;
  }

  public double getActualCollectorVelocity()
  {
    return currentCollectorVelocity;
  }

  public double getActualCollectorPosition()
  {
    return -collectMotor.getPosition().getValueAsDouble();
  }

  public double getRangeTOF1() {
    return tof1Range;
  }

  public void setRangeTOF1(double tof1Range) {
    this.tof1Range = tof1Range;
  }

  public double getRangeTOF2() {
    return tof2Range;
  }

  public void setRangeTOF2(double tof2Range) {
    this.tof2Range = tof2Range;
  }

  private void configureHardware(){
    TalonFXConfiguration collectConfigs = new TalonFXConfiguration();
    collectConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    collectConfigs.MotionMagic.MotionMagicCruiseVelocity = 20;
    collectConfigs.MotionMagic.MotionMagicAcceleration = 9500;
    collectConfigs.MotionMagic.MotionMagicJerk = 9500;
    collectMotor.getConfigurator().apply(collectConfigs, 0.5);

    //PID loop setting for collect motor
    var collectMotorClosedLoopConfig = new Slot0Configs();
    collectMotorClosedLoopConfig.withKP(collect_kP);
    collectMotorClosedLoopConfig.withKI(collect_kI);
    collectMotorClosedLoopConfig.withKD(collect_kD);
    collectMotorClosedLoopConfig.withKV(collect_kF);

    configError = collectMotor.getConfigurator().apply(collectMotorClosedLoopConfig, 0.5);
  }

  public boolean hasNote()
  {
    if (getRangeTOF1() < 0.45 || getRangeTOF2() < 0.2) // fill in Tof2 range value
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Target Velocity", this::getTargetCollectorVelocity, null);
    builder.addDoubleProperty("Actual Velocity", this::getActualCollectorVelocity, null);
    builder.addDoubleProperty("Commanded Velocity", this::getCommandedCollectorVelocity, null);
    builder.addDoubleProperty("tof1CollectorRange", this::getRangeTOF1, null);
    builder.addDoubleProperty("tof2CollectorRange", this::getRangeTOF2, null);
    builder.addBooleanProperty("Collector Has Note", this::hasNote, null);
    // builder.addBooleanProperty("ok", this::isOK, null);
    // builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
  }

  @Override
  public boolean updateDiagnostics() 
  {
    String result = "";
    boolean OK = true;

    if(!configError.isOK()){
      result += configError.getDescription();
      OK = false;
    }

    if(tof1DutyCycleInput.getFrequency() < 2 || collectMotorFault.hasFaults()){
      result += String.format("tof1 not working");
      OK = false;
    }
    
    if(tof2DutyCycleInput.getFrequency() < 2 || collectMotorFault.hasFaults()){
      result += String.format("tof2 not working");
      OK = false;
    }

    result += collectMotorFault.getFaults();

    return setDiagnosticsFeedback(result, OK);
  }
}
