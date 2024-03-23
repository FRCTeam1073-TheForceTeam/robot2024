// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Headlight extends SubsystemBase 
{
  PowerDistribution pd = new PowerDistribution(26, ModuleType.kRev);

  /** Creates a new Headlight. */
  public Headlight() {}

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void setHeadlight(boolean state)
  {
    pd.setSwitchableChannel(state);
  }

  public boolean getHeadlight()
  {
    return pd.getSwitchableChannel();
  }

  public void initSendable(SendableBuilder builder)
  {
    super.initSendable(builder);
    builder.addBooleanProperty("Headlight State", this::getHeadlight, this::setHeadlight);
  }
}
