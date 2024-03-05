// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorArm.POSE;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class CollectorIntakeCommand extends Command {
  Collector m_collector;
  CollectorArm m_collectorArm;
  Drivetrain m_drivetrain;
  OI m_OI;
  double minRange;
  double maxRange;
  double intakeRateThreshold;
  double vel;
  boolean isCollectable;
  boolean isCollected;
  double startCollectValue;

  double tofCurrentValue;
  double tofOldValue;

  double count = 0;

  /** Creates a new CollectorIntakeCommand. 
   * <p> This command is used for intaking into the collector from the ground
   * <p> This command ends when a note is in the collector
  */
  public CollectorIntakeCommand(Collector collector, CollectorArm collectorArm, Drivetrain ds) {
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
    double rate = 0;
    tofCurrentValue = m_collector.getRangeTOF(); 
    if(isCollectable){
      rate = (tofCurrentValue - tofOldValue) / 0.02; // calculating the rate of change of the TOF range

      if(rate < intakeRateThreshold){
        isCollectable = false;
      }
    }

    if(tofCurrentValue > maxRange){
      isCollectable = true;
      isCollected = false;
      count = 0;
    }

    
    
    //if(m_collector.getRangeTOF() < minRange){
    if(isCollectable){ // use the rate to decide when to stop
      //vel = (0.05 * Math.abs(m_drivetrain.getChassisSpeeds().vxMetersPerSecond)) + 3;
      vel = 3;
      m_collector.setTargetCollectorVelocity(-vel); //meters per sec
    }
    else{
      if(m_collectorArm.getPoseName() == POSE.AMP || m_collectorArm.getPoseName() == POSE.HANDOFF){
        m_collector.setTargetCollectorVelocity(-vel);
      }
      else{
        m_collector.setTargetCollectorVelocity(0);
      }
      isCollected = true;
    }

    if((isCollected && (count < 20))){ // check if it is collected and at the stow position and run for 50 loops
      m_collector.setTargetCollectorVelocity(0.5);
      count++;
    }

    tofOldValue = tofCurrentValue;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setTargetCollectorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(count == 20){
      return true;
    }
    else{
      return false;
    }
  }
}
