// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends Diagnostics {
  /** Creates a new Pivot. */

  private final TalonFX pivotMotor;
  private final MotorFault pivotMotorFault;
  private double pivotMotorPosition;

  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;

  public Pivot() {
    pivotMotor = new TalonFX(18); //Falcon - TODO: set CAN ID   
    pivotMotorFault = new MotorFault(pivotMotor, 18); //TODO set CAN ID
    pivotMotor.setPosition(0); //initialize to 0 rotations
    setConfigsPivot();
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
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runDiagnostics(){
    String result = "";
    this.setOK(true);
    if (
    pivotMotorFault.hasFaults());{
      this.setOK(false);
    }
  
    this.setDiagnosticResult(pivotMotorFault.getFaults());
  }
}
