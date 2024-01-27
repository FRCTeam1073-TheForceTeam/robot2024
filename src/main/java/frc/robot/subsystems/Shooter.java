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
import edu.wpi.first.wpilibj.DutyCycle;

public class Shooter extends Diagnostics{
  
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;
  private final TalonFX triggerMotorLeader;
  private final TalonFX triggerMotorFollower;
  private final TalonFX pivotMotor;
  private final MotorFault topShooterMotorFault;
  private final MotorFault bottomShooterMotorFault;
  private final MotorFault triggerMotorLeaderFault;
  private final MotorFault triggerMotorFollowerFault;
  private final MotorFault pivotMotorFault;
  private final DigitalInput shooterBeamBreak;
  private final DigitalInput toftrigger1;
  private double offset;
  private double leaderTriggerMotorVelocity;
  private double followerTriggerMotorVelocity;
  private double topShooterMotorVelocity;
  private double bottomShooterMotorVelocity;
  private double toftrigger1Freq;
  private double toftrigger1Range;
  private final double toftrigger1ScaleFactor = 100000;
  private DutyCycle toftrigger1DutyCycleInput;

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
    shooterBeamBreak = new DigitalInput(3); //TODO: correct port
    toftrigger1 = new DigitalInput(2)//TODO: correct port
    pivotMotor = new TalonFX(18); //Falcon - TODO: set CAN ID
    topShooterMotorFault = new MotorFault(topShooterMotor, 12); //TODO set CAN ID
    bottomShooterMotorFault = new MotorFault(bottomShooterMotor, 24); //TODO set CAN ID
    triggerMotorLeaderFault = new MotorFault(triggerMotorLeader, 18); //TODO set CAN ID   
    triggerMotorFollowerFault = new MotorFault(triggerMotorFollower, 16); //TODO set CAN ID   
    pivotMotorFault = new MotorFault(pivotMotor, 18); //TODO set CAN ID

    toftrigger1DutyCycleInput = new DutyCycle(toftrigger1);
    toftrigger1Freq = 0;
    toftrigger1Range = 0;    

    topShooterMotorVelocity = 0;
    bottomShooterMotorVelocity = 0;
    leaderTriggerMotorVelocity = 0;
    followerTriggerMotorVelocity = 0;
    offset = 0;
    pivotMotor.setPosition(0); //initialize to 0 rotations

    setConfigsShooter();
    setConfigsTrigger();
    setConfigsPivot();
}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    topShooterMotor.setControl(new VelocityVoltage(-topShooterMotorVelocity));
    bottomShooterMotor.setControl(new VelocityVoltage(bottomShooterMotorVelocity));
    triggerMotorLeader.setControl(new VelocityVoltage(leaderTriggerMotorVelocity));
    triggerMotorFollower.setControl(new VelocityVoltage(followerTriggerMotorVelocity));
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

@Override
public void initSendable(SendableBuilder builder)
{
  builder.setSmartDashboardType("Shooter");
  builder.addDoubleProperty("Top Motor Velocity", this::getTopShooterMotorVelocity, this::setTopShooterMotorVelocity);
  builder.addDoubleProperty("Bottom Motor Velocity", this::getBottomShooterMotorVelocity, this::setBottomShooterMotorVelocity);
  builder.addDoubleProperty("p", this::getP, this::setP);
  builder.addDoubleProperty("i", this::getI, this::setI);
  builder.addDoubleProperty("d", this::getD, this::setD);
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

  /* sets the desired top shooter motor velocity in rotations per second */
  public void setTopShooterMotorVelocity(double velocityRPS)
  {
    topShooterMotorVelocity = velocityRPS;
  }
  
  /* sets the desired bottom shooter motor velocity in rotations per second */
  public void setBottomShooterMotorVelocity(double velocityRPS)
  {
    bottomShooterMotorVelocity = velocityRPS;
  }

  /* sets the desired top trigger motor velocity in rotations per second */
  public void setTopTriggerMotorVelocity(double triggerMotorRPS)
  {
    leaderTriggerMotorVelocity = triggerMotorRPS;
  }

  /* sets the desired bottom trigger motor velocity in rotations per second */
  public void setBottomTriggerMotorVelocity(double triggerMotorRPS)
  {
    followerTriggerMotorVelocity = triggerMotorRPS;
  }

  /* sets the desired number of rotations for the pivot motor */
  public void setPivotMotorRotations(double pivotMotorPosition)
  {
    //todo next step
    pivotMotor.setControl(new PositionVoltage(pivotMotorPosition));
  }

  /* returns the position value */
  public double getPivotMotorRotations(){
    return pivotMotor.getPosition().getValue();
  }
  
  /* uses the beam break sensor to detect if the note has entered the trigger */
  public boolean noteIsInTrigger(){
    if (toftrigger1Range < 12){

    }
    return false;
    // return triggerBeamBreak.get();
  }

  /* uses the beam break sensor to detect if the note has entered the shooter */
  public boolean noteIsInShooter(){
    return shooterBeamBreak.get();
  }

/* gets the value of the velocity that the top shooter motor is at */
public double getTopShooterMotorVelocity(){
  return topShooterMotorVelocity;
}

/* gets the value of the velocity that the bottom shooter motor is at */
public double getBottomShooterMotorVelocity(){ 
  return bottomShooterMotorVelocity;
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

@Override
public void runDiagnostics(){
  String result = "";
  this.setOK(true);
  if (topShooterMotorFault.hasFaults()||
  bottomShooterMotorFault.hasFaults()||
  triggerMotorLeaderFault.hasFaults()||
  triggerMotorFollowerFault.hasFaults()||
  pivotMotorFault.hasFaults());{
    this.setOK(false);
  }

  if(toftrigger1DutyCycleInput.getFrequency()<2){
    result += String.format("toftrigger1 not working");
  }

  this.setDiagnosticResult(topShooterMotorFault.getFaults() + 
  bottomShooterMotorFault.getFaults() + 
  triggerMotorLeaderFault.getFaults() + 
  triggerMotorFollowerFault.getFaults() + 
  pivotMotorFault.getFaults()+ result);
}

}