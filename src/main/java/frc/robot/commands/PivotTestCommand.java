// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotTestCommand extends Command {
  /** Creates a new PivotTestCommand. */
  private Pivot pivot;
  private double pivotMotorPosition;
  private boolean isPivotOn;

  public PivotTestCommand(Pivot pivot) {
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivotMotorPosition = SmartDashboard.getNumber("Pivot Motor Rotations", 0.0);    
    // if(getPivotOn()){
    //   pivot.setPivotMotorPositionRadians(pivotMotorPosition);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  

  public boolean getPivotOn(){
    return isPivotOn;
  }

  public void setPivotOn(boolean pivoton){
    isPivotOn = pivoton;
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("Pivot Test Command");
    builder.addBooleanProperty("Toggle Pivot", this::getPivotOn, this::setPivotOn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
