// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Collector extends DiagnosticsSubsystem {

  private final String kCANbus = "CANivore";
  private TalonFX collectMotor = new TalonFX(14, kCANbus); // same thing
  private MotorFault collectMotorFault = new MotorFault(collectMotor, 14);
  private StatusCode configError;
  private double targetCollectorVelocity = 0;
  private double currentCollectorVelocity = 0;
  private double commandedCollectorVelocity = 0;
  public MotionMagicVelocityVoltage collectorVelocityVoltage;

  private DigitalInput tof1;
  private DutyCycle tof1DutyCycleInput;
  private double tof1DutyCycle;
  private double tof1Freq;
  private double tof1Range;
  private final double tofCollectorScaleFactor = 3000000/4; // for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million


  private final double collectorGearRatio = 12.0/18.0;
  private final double collectorWheelRadius = 0.0254; //meters
  private final double collectorMeterPerRotations = collectorWheelRadius * 2 * Math.PI * collectorGearRatio;
  
  private double collect_kP = 0.27;
  private double collect_kI = 0;
  private double collect_kD = 0.001;
  private double collect_kF = 0;

  /** Creates a new Collector. */
  public Collector() {

    tof1 = new DigitalInput(0); 
    tof1DutyCycleInput = new DutyCycle(tof1);
    tof1Freq = 0;
    tof1Range = 0;

    collectorVelocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0); 

    configureHardware();
  }

  @Override
  public void periodic() 
  {
    currentCollectorVelocity = collectMotor.getVelocity().getValueAsDouble() * collectorMeterPerRotations; //meters per second 
    runCollectMotor(targetCollectorVelocity);
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tofCollectorScaleFactor * (tof1DutyCycle / tof1Freq - 0.001) / 1000; //supposedly in meters

    updateDiagnostics();
  }
  
  /* Runs collector motor with input velocity */
  private void runCollectMotor(double vel)
  {
    collectMotor.setControl(collectorVelocityVoltage.withVelocity(vel / collectorMeterPerRotations)); //meters per rotation
  }

  public void setTargetCollectorVelocity(double velocity)
  {
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

  public double getRangeTOF() {
    return tof1Range;
  }

  public void setRangeTOF(double tof1Range) {
    this.tof1Range = tof1Range;
  }

  private void configureHardware(){
    TalonFXConfiguration collectConfigs = new TalonFXConfiguration();
    collectConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    collectConfigs.MotionMagic.MotionMagicCruiseVelocity = 20;
    collectConfigs.MotionMagic.MotionMagicAcceleration = 9000;
    collectConfigs.MotionMagic.MotionMagicJerk = 9000;
    collectMotor.getConfigurator().apply(collectConfigs, 0.5);

    //PID loop setting for collect motor
    var collectMotorClosedLoopConfig = new Slot0Configs();
    collectMotorClosedLoopConfig.withKP(collect_kP);
    collectMotorClosedLoopConfig.withKI(collect_kI);
    collectMotorClosedLoopConfig.withKD(collect_kD);
    collectMotorClosedLoopConfig.withKV(collect_kF);

    configError = collectMotor.getConfigurator().apply(collectMotorClosedLoopConfig, 0.5);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Target Velocity", this::getTargetCollectorVelocity, null);
    builder.addDoubleProperty("Actual Velocity", this::getActualCollectorVelocity, null);
    builder.addDoubleProperty("Commanded Velocity", this::getCommandedCollectorVelocity, null);
    builder.addDoubleProperty("tofCollectorRange", this::getRangeTOF, null);
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

    if(tof1DutyCycleInput.getFrequency()< 2 || collectMotorFault.hasFaults()){
      result += String.format("tof1 not working");
      OK = false;
    }

    result += collectMotorFault.getFaults();

    return setDiagnosticsFeedback(result, OK);
  }
}
