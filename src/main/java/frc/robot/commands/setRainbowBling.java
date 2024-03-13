// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Bling;



public class setRainbowBling extends Command {
  /** Creates a new rainbowBling. */
  private Bling bling;

  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  public int length = 48;
  

  public setRainbowBling(Bling bling, AddressableLED m_led, AddressableLEDBuffer m_ledBuffer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_led = m_led;
    this.m_ledBuffer = m_ledBuffer;
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    this.bling = bling;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bling.clearLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_led.setData(m_ledBuffer);
    bling.setRainbowBling();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    bling.clearLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
