// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Collector extends Diagnostics {
  TalonFX liftMotor = new TalonFX(0); // TODO: set device id
  TalonFX collectMotor = new TalonFX(0); // same thing
  private double speed;
  
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
    speed = 0;
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
  public void setLiftMotorSpeed(double speed)
  {
    liftMotor.setControl(new VelocityVoltage(speed));
  }

  public void setCollectMotorSpeed(double speed)
  {
    collectMotor.setControl(new VelocityVoltage(speed));
  }

  public void setMotorSpeeds(double speed)
  {
    setLiftMotorSpeed(speed);
    setCollectMotorSpeed(speed);
  }

  public void setSpeed(double speed)
  {
    this.speed = speed;
  }

  
  public double getSpeed()
  {
    return speed;
  }


  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Speed", this::getSpeed, this::setSpeed);
  }

  @Override
  public void periodic() 
  {
    setMotorSpeeds(speed);
  }
}
