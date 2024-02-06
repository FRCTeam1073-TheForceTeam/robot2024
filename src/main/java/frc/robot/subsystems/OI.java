// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;

public class OI extends Diagnostics{

  public Joystick operatorController;
  /** Creates a new OI. */
  public OI() {
    operatorController = new Joystick(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getXButton(){
    return operatorController.getRawButton(1);
  }
}
