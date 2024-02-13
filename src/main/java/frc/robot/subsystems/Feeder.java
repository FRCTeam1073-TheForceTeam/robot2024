// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO: may need position control/command (slot 1) to fire the note
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Feeder extends DiagnosticsSubsystem {
  private final TalonFX feederMotor;
  private final MotorFault feederMotorFault;
  private final DigitalInput toffeeder1;
  private SlewRateLimiter feederMotorFilter;
  /* 1 motor rotation = 2 wheel rotations
   * Diameter of the wheel is 2"
   * Wheel circumference is 2π (πd)
   * Therefore, the velocity = 4π inches/rotation
   */
  private double feederMetersPerRotation = 4 * Math.PI * 0.0254; // 0.0254 meters/inch
  
  private double targetFeederMotorVelocityRPS;
  private double feederMotorVelocityRPS;
  private double toffeeder1Freq;
  private double toffeeder1Range;
  private double toffeeder1DutyCycle;
  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;
  private final double toffeeder1ScaleFactor = 3000000/4; // for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million
  private final DutyCycle toffeeder1DutyCycleInput;
  /** Creates a new Trigger. */
  public Feeder() {
    feederMotor = new TalonFX(19); //Falcon
    toffeeder1 = new DigitalInput(1);
    feederMotorFault = new MotorFault(feederMotor, 19);
    
    toffeeder1DutyCycleInput = new DutyCycle(toffeeder1);
    toffeeder1Freq = 0;
    toffeeder1Range = 0;    

    targetFeederMotorVelocityRPS = 0;
    
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

  feederMotor.getConfigurator().apply(configs);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    toffeeder1Freq = toffeeder1DutyCycleInput.getFrequency();
    System.out.println(toffeeder1Freq);
    toffeeder1DutyCycle = toffeeder1DutyCycleInput.getOutput();
    toffeeder1Range = toffeeder1DutyCycleInput.getOutput();
    toffeeder1Range = (toffeeder1ScaleFactor * (toffeeder1DutyCycle / toffeeder1Freq - 0.001)) / 1000;
    System.out.println(toffeeder1Range);

    feederMotorVelocityRPS = feederMotorFilter.calculate(targetFeederMotorVelocityRPS);
    feederMotor.setControl(new VelocityVoltage(feederMotorVelocityRPS));
   }

  @Override
public void initSendable(SendableBuilder builder)
{
  builder.setSmartDashboardType("Shooter");
  builder.setSmartDashboardType("Tof");
  builder.addDoubleProperty("Range", this::getRangeTrigger1, null);
  builder.addDoubleProperty("Freq", this::getFreqTrigger1, null);
}

  /* sets the desired top trigger motor velocity in rotations per second */
public void setFeederMotorVelocity(double feederMotorMPS){
  targetFeederMotorVelocityRPS = feederMotorMPS / feederMetersPerRotation;
 }

/* gets the value of the trigger motor velocity */
public double getFeederMotorVelocityRPS(){
  return targetFeederMotorVelocityRPS;
}

public double getRangeTrigger1(){
  return toffeeder1Range;
}

public double getFreqTrigger1(){
  return toffeeder1Freq;
}


/* uses the beam break sensor to detect if the note has entered the trigger */
public boolean noteIsInTrigger(){
  if (toffeeder1Range < 12){

  }
  return false;
  // return triggerBeamBreak.get();
}

@Override
public boolean updateDiagnostics(){
  String result = "";
  boolean ok = true;
  if (feederMotorFault.hasFaults());{
    ok = false;
  }

  if(toffeeder1DutyCycleInput.getFrequency()<2){
    result += String.format("toffeeder1 not working");
  }
  result = feederMotorFault.getFaults();
  return setDiagnosticsFeedback(result, ok);
}
}
