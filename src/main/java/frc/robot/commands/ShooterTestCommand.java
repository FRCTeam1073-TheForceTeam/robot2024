// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTestCommand extends Command {
  /** Creates a new ShooterTestCommand. */
  private Shooter shooter;
  private OI oi;
  private double topShooterMotorVelocity;
  private double bottomShooterMotorVelocity;
  private boolean isShooterOn;

  public ShooterTestCommand(Shooter shooter, OI oi) {
    this.shooter = shooter;
    this.oi = oi;
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorRawButton(1)){
      shooter.setTopShooterMotorVelocity(0); //in MPS
      shooter.setBottomShooterMotorVelocity(0); //in MPS
    }
    if(oi.getOperatorRawButton(2)){
      shooter.setTopShooterMotorVelocity(10); //in MPS
      shooter.setBottomShooterMotorVelocity(10); //in MPS
    }
    
    // if(getShooterOn()){
    //   shooter.setTopShooterMotorVelocity(topShooterMotorVelocity);
    //   shooter.setBottomShooterMotorVelocity(bottomShooterMotorVelocity);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // public boolean getShooterOn(){
  //   return isShooterOn;
  // }

  // public void setShooterOn(boolean shooteron){
  //   isShooterOn = shooteron;
  // }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }}
