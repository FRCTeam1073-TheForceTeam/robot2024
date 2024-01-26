// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Collector extends Diagnostics {
  TalonFX liftMotor = new TalonFX(0); // TODO: set device id
  TalonFX collectMotor = new TalonFX(0); // same thing
  MotorFault liftMotorFault = new MotorFault(liftMotor, 0);
  MotorFault collectMotorFault = new MotorFault(liftMotor, 0);
  private double collectorSpeed;
  private double liftSpeed;
  private DigitalInput tof1;
  private DutyCycle tof1DutyCycleInput;
  private double tof1DutyCycle;
  private double tof1Freq;
  private double tof1Range;
  private final double tof1ScaleFactor = 100000; 

  private final double collectorTicksPerMeter = 1;
  // private final double intakeGearRatio = 5.0;
  // private final double intakeTicksPerRadian = 2048.0 * intakeGearRatio / (2.0 * Math.PI);
  
  // fill in PID values
  private double lift_kP = 0;
  private double lift_kI = 0;
  private double lift_kD = 0;
  private double lift_kF = 0;

  private double collect_kP = 0;
  private double collect_kI = 0;
  private double collect_kD = 0;
  private double collect_kF = 0;

  /** Creates a new Collector. */
  public Collector() {
    collectorSpeed = 0;
    liftSpeed = 0;

    tof1 = new DigitalInput(0); // TODO: set correct port #
    tof1DutyCycleInput = new DutyCycle(tof1);
    tof1Freq = 0;
    tof1Range = 0;
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
    var collectMotorClosedLoopConfig = new Slot0Configs();

    collectMotorClosedLoopConfig.withKP(collect_kP);
    collectMotorClosedLoopConfig.withKI(collect_kI);
    collectMotorClosedLoopConfig.withKD(collect_kD);
    collectMotorClosedLoopConfig.withKV(collect_kF);

    collectMotor.getConfigurator().apply(collectMotorClosedLoopConfig);

  }
  public void runLiftMotor(double liftSpeed)
  {
    liftMotor.setControl(new VelocityVoltage(liftSpeed));
  }

  public void runCollectMotor(double collectorSpeed)
  {
    collectMotor.setControl(new VelocityVoltage(collectorSpeed * collectorTicksPerMeter));
  }

  // public void setMotorSpeeds(double speed)
  // {
  //   setLiftMotorSpeed(speed);
  //   setCollectMotorSpeed(speed);
  // }


  public void setCollectorSpeed(double collectorSpeed)
  {
    this.collectorSpeed = collectorSpeed;
  }

  public double getCollectorSpeed()
  {
    return collectorSpeed;
  }

  public double getRange1() {
    return tof1Range;
  }

  public void setRange1(double tof1Range) {
    this.tof1Range = tof1Range;
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Speed", this::getCollectorSpeed, this::setCollectorSpeed);
    builder.addDoubleProperty("tof1Range", this::getRange1, this::setRange1);
    builder.addBooleanProperty("ok", this::isOK, null);
    builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
  }

  @Override
  public void periodic() 
  {
    runCollectMotor(collectorSpeed);
    runLiftMotor(liftSpeed);
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1ScaleFactor * (tof1DutyCycle / tof1Freq - 0.001);
    
  }


  @Override
  public void runDiagnostics() 
  {
    String result = "";
    setOK(true);

    if(liftMotorFault.hasFaults() || collectMotorFault.hasFaults()){
        setOK(false);
    }

    if(tof1DutyCycleInput.getFrequency()< 2){
      result += String.format("tof1 not working");
    }

    result += liftMotorFault.getFaults() + collectMotorFault.getFaults();

    setDiagnosticResult(result);
  }
}
