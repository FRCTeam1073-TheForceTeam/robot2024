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
  
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;
  private final TalonFX triggerMotorLeader;
  private final TalonFX triggerMotorFollower;
  private final TalonFX pivotMotor;
  private final DigitalInput triggerBeamBreak;
  private final DigitalInput shooterBeamBreak;
  private double offset;
  private double triggerMotorVelocity;
  private double topShooterMotorVelocity;
  private double bottomShooterMotorVelocity;

  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;

  /** Creates a new Shooter. **/
  public Shooter() {
    //one motor might be a follower
    topShooterMotor = new TalonFX(12); // Kracken - TODO: set CAN ID
    bottomShooterMotor = new TalonFX(24); //Kracken - TODO: set CAN ID
    triggerMotorLeader = new TalonFX(18); //Falcon - TODO: set CAN ID
    triggerMotorFollower = new TalonFX(16); //Falcon - TODO: set CAN ID
    triggerBeamBreak = new DigitalInput(0); //TODO: correct port
    shooterBeamBreak = new DigitalInput(3); //TODO: correct port
    pivotMotor = new TalonFX(18); //Falcon - TODO: set CAN ID
    topShooterMotorVelocity = 0;
    bottomShooterMotorVelocity = 0;
    offset = 0;
    setConfigsShooter();
    setConfigsTrigger();
    setConfigsPivot();
    pivotMotor.setPosition(0); //initialize to 0 rotations
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

  topShooterMotor.getConfigurator().apply(configs);
  bottomShooterMotor.getConfigurator().apply(configs);
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
  /* sets the desired top shooter motor velocity in rotations per second */
  public void setTopShooterMotorVelocity(double velocityRPS)
  {
    topShooterMotor.setControl(new VelocityVoltage(-velocityRPS));
  }
  
  /* sets the desired bottom shooter motor velocity in rotations per second */
  public void setBottomShooterMotorVelocity(double velocityRPS)
  {
    bottomShooterMotor.setControl(new VelocityVoltage(-velocityRPS));
  }

  /* sets the desired top trigger motor velocity in rotations per second */
  public void setTopTriggerMotorVelocity(double triggerMotorRPS)
  {
    triggerMotorLeader.setControl(new VelocityVoltage(triggerMotorRPS));
  }

  /* sets the desired bottom trigger motor velocity in rotations per second */
  public void setBottomTriggerMotorVelocity(double triggerMotorRPS)
  {
    triggerMotorFollower.setControl(new VelocityVoltage(triggerMotorRPS));
  }

  /* sets the desired number of rotations for the pivot motor */
  public void setPivotMotorRotations(double pivotMotorPosition)
  {
    pivotMotor.setControl(new PositionVoltage(pivotMotorPosition));
  }

  /* returns the position value */
  public double getPivotMotorRotations(){
    return pivotMotor.getPosition().getValue();
  }
  
  /* uses the beam break sensor to detect if the note has entered the trigger */
  public boolean noteIsInTrigger(){
    return triggerBeamBreak.get();
  }

  /* uses the beam break sensor to detect if the note has entered the shooter */
  public boolean noteIsInShooter(){
    return shooterBeamBreak.get();
  }

/* gets the value of the velocity that the top shooter motor is at */
public double getTopShooterMotorVelocity(){
  topShooterMotorVelocity = (topShooterMotor.getVelocity()).getValue();
  return topShooterMotorVelocity;
}

/* gets the value of the velocity that the bottom shooter motor is at */
public double getBottomShooterMotorVelocity(){ 
  bottomShooterMotorVelocity = bottomShooterMotor.getVelocity().getValue();
  return bottomShooterMotorVelocity;
}

/* sets the velocity of the trigger motor */
public void setTriggerMotorVelocity(double velocity) {
   triggerMotorVelocity = velocity;
}

/* gets the value of the trigger motor velocity */
public double getTriggerMotorVelocity(){
  return triggerMotorVelocity;
}

}