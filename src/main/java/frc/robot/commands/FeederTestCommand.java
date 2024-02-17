// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.OI;

public class FeederTestCommand extends Command {
  /** Creates a new TriggerTestCommand. */
  private Feeder feeder;
  private OI oi;
  private double feederMotorVelocityMPS;
  private boolean isFeederOn;

  public FeederTestCommand(Feeder feeder, OI oi) {
    this.feeder = feeder;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederMotorVelocityMPS = SmartDashboard.getNumber("Feeder Motor Velocity MPS", 0.0); //in MPS
    if(oi.getDriverRawButton(3)){
      feeder.setTargetFeederMotorVelocity(feederMotorVelocityMPS);
    }
    if(oi.getDriverRawButton(4)){
      feeder.setTargetFeederMotorVelocity(0); //in MPS
    }
    // if(getFeederOn()){
    //   feeder.setTargetFeederMotorVelocity(feederMotorVelocityMPS);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // public boolean getFeederOn(){
  //   return isFeederOn;
  // }

  // public void setFeederOn(boolean feederon){
  //   isFeederOn = feederon;
  // }


  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("Feeder Test Command");
    // builder.addBooleanProperty("Toggle Feeder", this::getFeederOn, this::setFeederOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
