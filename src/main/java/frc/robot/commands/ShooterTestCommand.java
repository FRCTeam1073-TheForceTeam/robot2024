// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Trigger;

public class ShooterTestCommand extends Command {
  /** Creates a new ShooterTestCommand. */
  private Shooter shooter;
  private Trigger trigger;
  private OI oi;
  private Pivot pivot;
  private double topShooterMotorVelocity;
  private double bottomShooterMotorVelocity;
  private double triggerMotorLeaderVelocity;
  private double triggerMotorFollowerVelocity;
  private double pivotMotorPosition;
  private boolean isTriggerOn;
  private boolean isShooterOn;
  private boolean isPivotOn;
  public ShooterTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    topShooterMotorVelocity = SmartDashboard.getNumber("Top Shooter Motor Velocity", 0.0); //in RPS
    bottomShooterMotorVelocity = SmartDashboard.getNumber("Bottom Shooter Motor Velocity", 0.0); //in RPS
    triggerMotorLeaderVelocity = SmartDashboard.getNumber("Trigger Motor Leader Velocity", 0.0); //in RPS
    triggerMotorFollowerVelocity = SmartDashboard.getNumber("Trigger Motor Follower Velocity", 0.0); //in RPS
    pivotMotorPosition = SmartDashboard.getNumber("Pivot Motor Rotations", 0.0);
    if(getTriggerOn()){
      trigger.setTopTriggerMotorVelocity(triggerMotorLeaderVelocity);
      trigger.setBottomTriggerMotorVelocity(triggerMotorFollowerVelocity);
    }
    if(getShooterOn()){
      shooter.setTopShooterMotorVelocity(topShooterMotorVelocity);
      shooter.setBottomShooterMotorVelocity(bottomShooterMotorVelocity);
    }
    if(getPivotOn()){
      pivot.setPivotMotorRotations(pivotMotorPosition);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public boolean getTriggerOn(){
    return isTriggerOn;
  }

  public void setTriggerOn(boolean triggeron){
    isTriggerOn = triggeron;
  }

  public boolean getShooterOn(){
    return isShooterOn;
  }

  public void setShooterOn(boolean shooteron){
    isShooterOn = shooteron;
  }

  public boolean getPivotOn(){
    return isPivotOn;
  }

  public void setPivotOn(boolean pivoton){
    isPivotOn = pivoton;
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("Shooter Test Command");
    builder.addBooleanProperty("Toggle Trigger", this::getTriggerOn, this::setTriggerOn);
    builder.addBooleanProperty("Toggle Shooter", this::getShooterOn, this::setShooterOn);
    builder.addBooleanProperty("Toggle Pivot", this::getPivotOn, this::setPivotOn);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
