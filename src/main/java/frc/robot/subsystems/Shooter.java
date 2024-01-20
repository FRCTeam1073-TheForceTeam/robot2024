// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// need to deal with angle adjustments, imputting velocity or power //

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput; 

public class Shooter extends Diagnostics{
  
  private final TalonFX shooterMotorLeader;
  private final TalonFX shooterMotorFollower;
  private final TalonFX feederMotorLeader;
  private final TalonFX feederMotorFollower;
  private final DigitalInput feederBeamBreakEnter;
  private final DigitalInput feederBeamBreakExit;
  private double shooterMotorVelocity;
  private double feederMotorVelocity;

  /** Creates a new Shooter. */
  public Shooter() {
    //one motor might be a follower
    shooterMotorLeader = new TalonFX(12); // Krackin - TODO: set CAN ID
    shooterMotorFollower = new TalonFX(24); //Krackin - TODO: set CAN ID
    feederMotorLeader = new TalonFX(18); //Falcon - TODO: set CAN ID
    feederMotorFollower = new TalonFX(16); //Falcon - TODO: set CAN ID
    feederBeamBreakEnter = new DigitalInput(0); //TODO: correct port
    feederBeamBreakExit = new DigitalInput(3); //TODO: correct port
}
  

  public boolean feederBeamBreakEnterValue(){
    return feederBeamBreakEnter.get();
  }

  public boolean feederBeamBreakExitValue(){
    return feederBeamBreakExit.get();
  }

  /* return true if note is only in the first beam break sensor */
  public boolean noteEnteringFeeder(){
    return feederBeamBreakEnterValue() & ! feederBeamBreakExitValue(); 
  }


  /* return true if note is only in the second beam break sensor */
  public boolean noteExitingFeeder(){
    return !feederBeamBreakEnterValue() & feederBeamBreakExitValue(); 
  }

  /* return true if note is in both beam break sensors */
  public boolean noteInFeeder(){
    return feederBeamBreakEnterValue() & feederBeamBreakExitValue(); 
  }

public void setShooterMotorVelocity(double velocity) {
   shooterMotorVelocity = velocity;
}

public double getShooterMotorVelocity(){
  return shooterMotorVelocity;
}

public void setFeederMotorVelocity(double velocity) {
   feederMotorVelocity = velocity;
}

public double getFeederMotorVelocity(){
  return feederMotorVelocity;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }}