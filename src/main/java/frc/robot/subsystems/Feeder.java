// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends DiagnosticsSubsystem {
  private final TalonFX triggerMotorLeader;
  private final TalonFX triggerMotorFollower;
  private final MotorFault triggerMotorLeaderFault;
  private final MotorFault triggerMotorFollowerFault;
  private final DigitalInput toftrigger1;
  
  private double leaderTriggerMotorVelocity;
  private double followerTriggerMotorVelocity;
  private double toftrigger1Freq;
  private double toftrigger1Range;
  private double toftrigger1DutyCycle;
  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;
  private final double toftrigger1ScaleFactor = 3000000/4; // for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million
  private final DutyCycle toftrigger1DutyCycleInput;
  /** Creates a new Trigger. */
  public Feeder() {
    triggerMotorLeader = new TalonFX(18); //Falcon - TODO: set CAN ID
    triggerMotorFollower = new TalonFX(16); //Falcon - TODO: set CAN ID
    toftrigger1 = new DigitalInput(2);//TODO: correct port/channel
    triggerMotorLeaderFault = new MotorFault(triggerMotorLeader, 18); //TODO set CAN ID   
    triggerMotorFollowerFault = new MotorFault(triggerMotorFollower, 16); //TODO set CAN ID
    
    toftrigger1DutyCycleInput = new DutyCycle(toftrigger1);
    toftrigger1Freq = 0;
    toftrigger1Range = 0;    

    leaderTriggerMotorVelocity = 0;
    followerTriggerMotorVelocity = 0;
    
    setConfigsTrigger();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    toftrigger1Freq = toftrigger1DutyCycleInput.getFrequency();
    System.out.println(toftrigger1Freq);
    toftrigger1DutyCycle = toftrigger1DutyCycleInput.getOutput();
    toftrigger1Range = toftrigger1DutyCycleInput.getOutput();
    toftrigger1Range = toftrigger1ScaleFactor * (toftrigger1DutyCycle / toftrigger1Freq - 0.001);
    System.out.println(toftrigger1Range);
    triggerMotorLeader.setControl(new VelocityVoltage(leaderTriggerMotorVelocity));
    triggerMotorFollower.setControl(new VelocityVoltage(followerTriggerMotorVelocity));
  }

  @Override
public void initSendable(SendableBuilder builder)
{
  builder.setSmartDashboardType("Shooter");
  builder.addDoubleProperty("p", this::getP, this::setP);
  builder.addDoubleProperty("i", this::getI, this::setI);
  builder.addDoubleProperty("d", this::getD, this::setD);
  builder.setSmartDashboardType("Tof");
  builder.addDoubleProperty("Range", this::getRangeTrigger1, null);
  builder.addDoubleProperty("Freq", this::getFreqTrigger1, null);
}

public double getP(){
  return p;
}

public void setP(double p){
  this.p = p;
}

public double getI(){
  return i;
}

public void setI(double i){
  this.i = i;
}

public double getD(){
  return d;
}

public void setD(double d){
  this.d = d;
}

  /* sets the desired top trigger motor velocity in rotations per second */
public void setTopTriggerMotorVelocity(double triggerMotorRPS){
  leaderTriggerMotorVelocity = triggerMotorRPS;
 }

/* sets the desired bottom trigger motor velocity in rotations per second */
public void setBottomTriggerMotorVelocity(double triggerMotorRPS){
  followerTriggerMotorVelocity = triggerMotorRPS;
}

/* gets the value of the trigger motor velocity */
public double getLeaderTriggerMotorVelocity(){
  return leaderTriggerMotorVelocity;
}

public double getFollowerTriggerMotorVelocity(){
  return followerTriggerMotorVelocity;
}

public double getRangeTrigger1(){
  return toftrigger1Range;
}

public double getFreqTrigger1(){
  return toftrigger1Freq;
}


/* uses the beam break sensor to detect if the note has entered the trigger */
public boolean noteIsInTrigger(){
  if (toftrigger1Range < 12){

  }
  return false;
  // return triggerBeamBreak.get();
}

@Override
public boolean updateDiagnostics(){
  String result = "";
  boolean ok = true;
  if (triggerMotorLeaderFault.hasFaults()||
  triggerMotorFollowerFault.hasFaults());{
    ok = false;
  }

  if(toftrigger1DutyCycleInput.getFrequency()<2){
    result += String.format("toftrigger1 not working");
  }
  result = triggerMotorLeaderFault.getFaults() + triggerMotorFollowerFault.getFaults();
  return setDiagnosticsFeedback(result, ok);
}
}
