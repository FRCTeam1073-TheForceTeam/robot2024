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
  /** Creates a new Pivot. */

  private final TalonFX pivotMotor;
  private final MotorFault pivotMotorFault;
  private final SlewRateLimiter pivotMotorFilter;
  private double pivotMotorPosition;
  private double targetPivotMotorPositionRot;
  private double pivotRotationsPerRadian = 20.0; //TODO: get this from EM
  private PositionVoltage pivotPositionVoltage;

  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;

  public Pivot() {
    pivotMotor = new TalonFX(21, "CANivore"); //Falcon  
    pivotMotorFault = new MotorFault(pivotMotor, 21);
    pivotMotorFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    pivotPositionVoltage = new PositionVoltage(0);
        // TODO - is this right
    pivotMotor.setPosition(0); //TODO - initialize position
    setConfigsPivot();

    targetPivotMotorPositionRot = 0;
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
  //configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

  pivotMotor.getConfigurator().apply(configs);
}

  /* sets the desired number of rotations for the pivot motor */
  public void setPivotMotorPositionRadians(double pivotMotorPositionRad)
  {
    //todo next step
    targetPivotMotorPositionRot  = pivotMotorPositionRad * pivotRotationsPerRadian;
  }
  /* returns the position value */
  public double getPivotMotorPositionRadians(){
    return pivotMotor.getPosition().getValue() / pivotRotationsPerRadian;
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotMotorPosition = pivotMotorFilter.calculate(-targetPivotMotorPositionRot);
    pivotMotor.setControl(pivotPositionVoltage.withPosition(pivotMotorPosition));
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Pivot");
    builder.addDoubleProperty("Pivot Motor Velocity", this::getPivotMotorPositionRadians, this::setPivotMotorPositionRadians);
  }

  @Override
  public boolean updateDiagnostics(){
    String result = "";
    boolean ok = true;
    if (
    pivotMotorFault.hasFaults());{
      ok = false;
    }
    result = pivotMotorFault.getFaults();
    return setDiagnosticsFeedback(result, ok);
  }
}
