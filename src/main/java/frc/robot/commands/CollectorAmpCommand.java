// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;

public class CollectorAmpCommand extends Command {

  Collector m_collector;
  CollectorArm m_collectorArm;
  Drivetrain m_drivetrain;
  double minRange;
  double maxRange;
  double intakeRateThreshold;
  double vel;
  boolean isCollectable;
  boolean isCollected;
  double startCollectValue;

  double tofCurrentValue;
  double tofOldValue;

  /** Creates a new CollectorAmpCommand. 
   * 
  */
  public CollectorAmpCommand(Collector collector, CollectorArm collectorArm, Drivetrain ds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collector;
    m_collectorArm = collectorArm;
    m_drivetrain = ds;
    minRange = 0.4;
    maxRange = 0.72;
    intakeRateThreshold = 0.001;
    vel = 0;
    isCollectable = true;
    startCollectValue = 0;

    tofCurrentValue = 0;
    tofOldValue = 0;

    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tofCurrentValue = m_collector.getRangeTOF(); 
    
    //outtake
    if(tofCurrentValue < maxRange){
      vel = -6;
      m_collector.setTargetCollectorVelocity(vel); //meters per sec
    }
    else{
      m_collector.setTargetCollectorVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setTargetCollectorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tofCurrentValue > maxRange){
      return true;
    }
    else{
      return false;
    }
  }
}
