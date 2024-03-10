// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.CollectorArm.POSE;

public class CollectorTeleop extends Command {
  /** Creates a new CollectorTeleop. */
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
  double extraIntakeCounter = 2;


  public CollectorTeleop(Collector collector, CollectorArm collectorArm, Drivetrain ds, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collector;
    m_collectorArm = collectorArm;
    m_drivetrain = ds;
    m_OI = oi;
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
      extraIntakeCounter = 2;
    }
    
    if(m_OI.getOperatorRawButton(5)) //outtake
    {
      vel = 10;
      m_collector.setTargetCollectorVelocity(vel); //meters per sec

      if(tofCurrentValue > maxRange){
        m_collector.setTargetCollectorVelocity(0);
      }
    }
    else if(m_OI.getOperatorRawButton(6)) //intake
    {
      vel = (0.05 * Math.abs(m_drivetrain.getChassisSpeeds().vxMetersPerSecond)) + 15;
      m_collector.setTargetCollectorVelocity(-vel); //meters per sec

      if(tofCurrentValue > maxRange){
        isCollected = false;
        count = 0;
      }
      
      //if(m_collector.getRangeTOF() < minRange){
      if(!isCollectable){ // use the rate to decide when to stop
        if(m_collectorArm.getPoseName() == POSE.AMP){
          vel = 25;
          m_collector.setTargetCollectorVelocity(-vel);
        }
        else{
          if(extraIntakeCounter == 0){
            m_collector.setTargetCollectorVelocity(0);
            isCollected = true;
          }
          else{
            extraIntakeCounter--;
          }
        }
      }
    }
    else {
      m_collector.setTargetCollectorVelocity(0);
    }

    if((isCollected && (count < 20) && (m_collectorArm.getPoseName() == POSE.STOW))){ // check if it is collected and at the stow position and run for 50 loops
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
    return false;
  }
}
