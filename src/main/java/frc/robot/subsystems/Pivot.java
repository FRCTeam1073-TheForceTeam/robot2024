// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO: check slew rate limiters, initialize to the right angle, 
change methods to use pivot in radians */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Pivot extends DiagnosticsSubsystem {

  // Motor and related objects
  private final TalonFX pivotMotor;
  private final MotorFault pivotMotorFault;
  private final SlewRateLimiter pivotMotorFilter;

  // Motor scale factors
  private final double pivotGearRatio = 8.0/1.0;
  private final double pivotWheelRadius = 0.1429; //meters
  private final double pivotMeterPerRotations = pivotWheelRadius * 2 * Math.PI * pivotGearRatio;
  private double pivotRotationsPerRadian = 8.181818 / (2 * Math.PI);

  // Motor pid values
  private double p = 0.0;
  private double i = 0.0;
  private double d = 0.0;

  // Position variables in rotations
  private double targetPositionRad;
  private double commandedPositionRad;
  private double currentPositionRad;

  // CANbus for this subsystem
  private final String kCANbus = "CANivore";

  /** Creates a new Pivot. */
  public Pivot() {
    //pivotMotor = new TalonFX(21, kCANbus); //Falcon
    pivotMotor = new TalonFX(21);
    pivotMotorFault = new MotorFault(pivotMotor, 21);
    pivotMotorFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    pivotMotor.setPosition(0); //TODO - initialize position

    targetPositionRad = 0;

    configureHardware();
  }

  @Override
  public void periodic() {
    updateFeedback();
    // This method will be called once per scheduler run
    commandedPositionRad = pivotMotorFilter.calculate(-targetPositionRad);
    pivotMotor.setControl(new PositionVoltage(commandedPositionRad * pivotRotationsPerRadian));
  }

  /* Updates the current motor position */
  public void updateFeedback(){
    currentPositionRad = pivotMotor.getPosition().getValue();
  }

  /* Sets the desired motor position in radians */
  public void setTargetPivotPositionInRad(double pivotMotorPositionRad)
  {
    targetPositionRad  = pivotMotorPositionRad;
  }

  /* Gets the desired motor position in radians */
  public double getTargetPositionInRad()
  {
    return targetPositionRad;
  }

  /* Gets the ratelimited commanded velocity for the motor in rotations per second */
  public double getCommandedPositionInRad(){
    return commandedPositionRad;
  }

  /* Gets the actual reported rotation of the motor in radians */
  public double getCurrentPositionInRad(){
    return currentPositionRad;
  }

  public void configureHardware(){
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
    //configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var error = pivotMotor.getConfigurator().apply(configs);
    if (!error.isOK()) 
    {
      System.err.println(String.format("PIVOT MOTOR ERROR: %s", error.toString()));
      setDiagnosticsFeedback(error.getDescription(), false);
    }
  }

  @Override
  public boolean updateDiagnostics(){
    String result = getDiagnosticsDetails();
    boolean ok = diagnosticsOk();
    if (
    pivotMotorFault.hasFaults());{
      ok = false;
    }
    result = pivotMotorFault.getFaults();
    return setDiagnosticsFeedback(result, ok);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Pivot");
    //builder.addDoubleProperty("Target Pivot Motor Position", this::getTargetPivotPositionInRad, this::setTargetPivotPositionInRad);
    builder.addDoubleProperty("Commanded Pivot Motor Position", this::getCommandedPositionInRad, null);
    builder.addDoubleProperty("Actual Pivot Motor Position", this::getCurrentPositionInRad, null);
  }
}
