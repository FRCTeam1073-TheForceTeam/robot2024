// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// need to deal with angle adjustments, imputting velocity or power //

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DigitalInput; 
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends Diagnostics{
  
  TalonFX topMotor;
  TalonFX bottomMotor;
  private final TalonFX triggerMotorLeader;
  private final TalonFX triggerMotorFollower;
  private final TalonFX pivotMotor;
  private final DigitalInput triggerBeamBreakEnter;
  private final DigitalInput triggerBeamBreakExit;
  /*TODO: One near motor for shooter, one near motor for trigger?*/
  private double shooterMotorVelocity;
  private double offset;
  private double triggerMotorVelocity;
  private double topMotorVelocity;
  private double bottomMotorVelocity;

  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;

  /** Creates a new Shooter. **/
  public Shooter() {
    //one motor might be a follower
    topMotor = new TalonFX(12); // Kracken - TODO: set CAN ID
    bottomMotor = new TalonFX(24); //Kracken - TODO: set CAN ID
    triggerMotorLeader = new TalonFX(18); //Falcon - TODO: set CAN ID
    triggerMotorFollower = new TalonFX(16); //Falcon - TODO: set CAN ID
    triggerBeamBreakEnter = new DigitalInput(0); //TODO: correct port
    triggerBeamBreakExit = new DigitalInput(3); //TODO: correct port
    pivotMotor = new TalonFX(18); //Falcon - TODO: set CAN ID
    shooterMotorVelocity = 0;
    offset = 0;
    setConfigsShooter();
    setConfigsTrigger();
}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

/* returns true if the beam has been broken, false if there's no note in the sensor */

public void setConfigsShooter(){
  TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.Slot0.kP = p;
  configs.Slot0.kI = i;
  configs.Slot0.kD = d;
  configs.Slot0.kV = 0.12;
  configs.Voltage.PeakForwardVoltage = 8;
  configs.Voltage.PeakReverseVoltage = -8;

  configs.Slot1.kP = 5;
  configs.Slot1.kI = 0.1;
  configs.Slot1.kD = 0.001;

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

  topMotor.getConfigurator().apply(configs);
  bottomMotor.getConfigurator().apply(configs);
}

public void setConfigsTrigger(){
  TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.Slot0.kP = p;
  configs.Slot0.kI = i;
  configs.Slot0.kD = d;
  configs.Slot0.kV = 0.12;
  configs.Voltage.PeakForwardVoltage = 8;
  configs.Voltage.PeakReverseVoltage = -8;

  configs.Slot1.kP = 5;
  configs.Slot1.kI = 0.1;
  configs.Slot1.kD = 0.001;

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

  triggerMotorLeader.getConfigurator().apply(configs);
  triggerMotorFollower.getConfigurator().apply(configs);
}

public void setConfigsPivot(){
  TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.Slot0.kP = p;
  configs.Slot0.kI = i;
  configs.Slot0.kD = d;
  configs.Slot0.kV = 0.12;
  configs.Voltage.PeakForwardVoltage = 8;
  configs.Voltage.PeakReverseVoltage = -8;

  configs.Slot1.kP = 5;
  configs.Slot1.kI = 0.1;
  configs.Slot1.kD = 0.001;

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

  pivotMotor.getConfigurator().apply(configs);
}

  public void setTopMotorVelocity(double velocityRPS)
  {
    topMotorVelocity = velocityRPS;
    topMotor.setControl(new VelocityVoltage(-velocityRPS));
  }
  
    public void setBottomMotorVelocity(double velocityRPS)
  {
    bottomMotorVelocity = velocityRPS;
    bottomMotor.setControl(new VelocityVoltage(-velocityRPS));
  }
/* TODO: Rename */

    public void setTriggerMotorTopVelocity(double triggerMotorRPS)
  {
    triggerMotorLeader.setControl(new VelocityVoltage(triggerMotorRPS));
  }
    public void setTriggerMotorBottomVelocity(double triggerMotorRPS)
  {
    triggerMotorFollower.setControl(new VelocityVoltage(triggerMotorRPS));
  }

    public void setPivotMotor(double pivotMotorPosition)
  {
    pivotMotor.setControl(new PositionVoltage(pivotMotorPosition));
  }
  
  public boolean noteIsInTrigger(){
    return triggerBeamBreakEnter.get();
  }

  public boolean noteIsInShooter(){
    return triggerBeamBreakExit.get();
  }

public double getShooterMotorVelocity(){
  return shooterMotorVelocity;
}

public void setTriggerMotorVelocity(double velocity) {
   triggerMotorVelocity = velocity;
}

public double getTriggerMotorVelocity(){
  return triggerMotorVelocity;
}

}