// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Arm extends Diagnostics {
  
  enum POSE{
    START,
    STOW,
    COLLECT,
    SCORE
  }

  TalonFX liftMotor = new TalonFX(0); // TODO: set device id
  TalonFX extendMotor = new TalonFX(0); // TODO: set device id
  
  MotorFault liftMotorFault = new MotorFault(liftMotor, 0);
  MotorFault extendMotorFault = new MotorFault(extendMotor, 0);
  private double liftSpeed;
  private double extendSpeed; 

  private final double liftTicksPerMeter = 1;
  
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
  public Arm() {
    liftSpeed = 0;
    extendSpeed = 0;
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
  public void runLiftMotor(double liftSpeed)
  {
    liftMotor.setControl(new VelocityVoltage(liftSpeed * liftTicksPerMeter));
  }

  public void runExtendMotor(double extendSpeed)
  {
    extendMotor.setControl(new VelocityVoltage(extendSpeed));
  }

  public void setExtendSpeed(double extendSpeed)
  {
    this.extendSpeed = extendSpeed;
  }

  public double getExtendSpeed()
  {
    return extendSpeed;
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Speed", this::getExtendSpeed, this::setExtendSpeed);
    builder.addBooleanProperty("ok", this::isOK, null);
    builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
    extendMotor.initSendable(builder);
    liftMotor.initSendable(builder);
  }

  @Override
  public void periodic() 
  {
    runExtendMotor(extendSpeed);
    runLiftMotor(liftSpeed);
  }


  @Override
  public void runDiagnostics() 
  {
    String result = "";
    setOK(true);

    if(liftMotorFault.hasFaults() || extendMotorFault.hasFaults()){
        setOK(false);
    }

    result += liftMotorFault.getFaults() + extendMotorFault.getFaults();

    setDiagnosticResult(result);
  }
}
