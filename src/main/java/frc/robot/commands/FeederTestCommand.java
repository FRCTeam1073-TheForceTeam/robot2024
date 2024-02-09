// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederTestCommand extends Command {
  /** Creates a new TriggerTestCommand. */
  private Feeder feeder;
  private double feederMotorVelocityMPS;
  private boolean isTriggerOn;
  public FeederTestCommand(Feeder feeder) {
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederMotorVelocityMPS = SmartDashboard.getNumber("Feeder Motor Velocity MPS", 0.0); //in MPS
    if(getTriggerOn()){
      feeder.setFeederMotorVelocity(feederMotorVelocityMPS);
    }}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public boolean getTriggerOn(){
    return isTriggerOn;
  }

  public void setTriggerOn(boolean triggeron){
    isTriggerOn = triggeron;
  }


  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("Trigger Test Command");
    builder.addBooleanProperty("Toggle Trigger", this::getTriggerOn, this::setTriggerOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
