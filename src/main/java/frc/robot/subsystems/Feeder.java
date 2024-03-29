// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO: may need position control/command (slot 1) to fire the note
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Feeder extends DiagnosticsSubsystem {

  // Motor and related objects
  private final TalonFX feederMotor;
  private final MotorFault feederMotorFault;
  private SlewRateLimiter feederMotorLimiter;
  private StatusCode configError;

  // Motor scale factor
  private double feederMetersPerRotation = Math.PI * 0.0254; // 0.0254 meters/inch
    /* 1 wheelrot   2pi inches   0.0254 meters   0.0254pi meters
     * ---------- * ---------- * ------------- = ---------------
     * 2 motorrot   1 wheelrot        inch          motorrot
     */

  // Motor pid values
  private double p = 0.2;
  private double i = 0.0;
  private double d = 0.003;

  // Velocity variables
  private double targetFeederVelocityMPS;
  private double commandedVelocityMPS;
  private double currentVelocityMPS;

  // Time of flight sensor
  private final DigitalInput feederTof;
  private final DutyCycle feederTofDutyCycleInput;
  private final double feederTofScaleFactor = 3000000/4;
    // Scale factor: for 50cm (irs16a): 3/4 million || for 130 cm (irs17a): 2 million || for 300 cm (irs17a): 4 million
  private double feederTofFreq;
  private double feederTofRange;
  private double feederTofDutyCycle;

  // VelocityVoltage object
  private VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

  // CANbus for this subsystem
  private final String kCANbus = "CANivore";

  /** Creates a new Trigger. */
  public Feeder() {
    super.setSubsystem("Feeder");
    feederMotor = new TalonFX(19, kCANbus);
    feederMotorFault = new MotorFault(feederMotor, 19);
    feederMotorLimiter = new SlewRateLimiter(13); //limits the rate of change to 0.5 units per seconds

    feederTof = new DigitalInput(1);
    feederTofDutyCycleInput = new DutyCycle(feederTof);
    feederTofFreq = 0;
    feederTofRange = 0;

    targetFeederVelocityMPS = 0;
    currentVelocityMPS = 0;
    
    configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateDiagnostics();
    updateFeedback();

    feederTofFreq = feederTofDutyCycleInput.getFrequency();
    feederTofDutyCycle = feederTofDutyCycleInput.getOutput();
    feederTofRange = feederTofDutyCycleInput.getOutput();
    feederTofRange = (feederTofScaleFactor * (feederTofDutyCycle / feederTofFreq - 0.001)) / 1000;

    // Calculates ratelimited velocity in rotations per second based on meters/second target velocity and runs the motor
    commandedVelocityMPS = (feederMotorLimiter.calculate(-targetFeederVelocityMPS));
    feederMotor.setControl(feederVelocityVoltage.withVelocity(commandedVelocityMPS / feederMetersPerRotation));
  }

  /* Updates the current motor velocity */
  public void updateFeedback(){
    currentVelocityMPS = feederMotor.getVelocity().getValue() * feederMetersPerRotation;
  }

  /* Sets the target motor velocity given in meters per second */
  public void setTargetVelocityInMPS(double feederMotorMPS){
    targetFeederVelocityMPS = feederMotorMPS;
  }

  /* Gets the target velocity for the motor in meters per second */
  public double getTargetVelocityInMPS(){
    return targetFeederVelocityMPS;
  }

  /* Gets the ratelimited commanded velocity for the motor in rotations per second */
  public double getCommandedVelocityInMPS(){
    return commandedVelocityMPS;
  }

  /* Gets the actual reported velocity of the motor in rotations per second */
  public double getCurrentVelocityInMPS(){
    return currentVelocityMPS;
  }

  /* Gets the time of flight range */
  public double getTofRange(){
    return feederTofRange;
  }

  /* Gets the time of flight frequency */
  public double getTofFreq(){
    return feederTofFreq;
  }

  /* Uses the beam break sensor to detect if the note has entered the trigger */
  public boolean noteIsInTrigger(){
    if (feederTofRange < 12){

    }
    return false;
    // return triggerBeamBreak.get();
  }

  public void configureHardware(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = p;
    configs.Slot0.kI = i;
    configs.Slot0.kD = d;
    configs.Slot0.kV = 0.12;
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    configError = feederMotor.getConfigurator().apply(configs);
  }

  @Override
  public boolean updateDiagnostics(){
    String result = "";
    boolean ok = true;
    if (feederMotorFault.hasFaults()){
      ok = false;
    }
    if(!ok){
      result += feederMotorFault.getFaults();
    }

    if(feederTofDutyCycleInput.getFrequency()<2){
      ok = false;
      result += String.format("toffeeder1 not working");
    }
    if (!configError.isOK()) 
    {
      System.err.println(String.format("FEEDER MOTOR ERROR: %s", configError.toString()));
      result += configError.getDescription();
      ok = false;
    }
    return setDiagnosticsFeedback(result, ok);
  }

  public boolean hasNote()
  {
    if (getTofRange() < 0.2)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    super.initSendable(builder);
    builder.addDoubleProperty("Tof Range", this::getTofRange, null);
    builder.addDoubleProperty("Tof Freq", this::getTofFreq, null);
    builder.addDoubleProperty("Target Feeder Velocity", this::getTargetVelocityInMPS, this::setTargetVelocityInMPS);
    builder.addDoubleProperty("Commanded Feeder Velocity", this::getCommandedVelocityInMPS, null);
    builder.addDoubleProperty("Actual Feeder Velocity", this::getCurrentVelocityInMPS, null);
    builder.addBooleanProperty("Feeder Has Note", this::hasNote, null);
  }
}