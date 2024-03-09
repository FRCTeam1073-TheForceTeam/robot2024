// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class ShooterSequences extends SequentialCommandGroup{
    
  public ShooterSequences(){ //Shooter shooter, Feeder feeder, RangeFinder rangefinder

    }

  public SequentialCommandGroup shootToSpeaker(HandoffSequence m_handoff, CollectorArm m_collectorArm, Collector m_collector, Shooter m_shooter, Feeder m_feeder, Pivot m_pivot, RangeFinder m_rangeFinder){
    switch(m_collectorArm.getPoseName()) {
      case STOW:
        return new SequentialCommandGroup(
          m_handoff.handoff(m_collector, m_collectorArm, m_feeder, m_pivot),
          new ParallelCommandGroup(
            new PivotToRangeCommand(m_pivot, m_rangeFinder, true),
            new ShootAtRangeCommand(m_shooter, m_rangeFinder) //, m_rangefinder.getRange()),
          ),
          new ParallelCommandGroup(
            new FeederShootCommand(m_feeder, 30),
            new ShooterWatchForNoteCommand(m_shooter)
          ),
          new ParallelCommandGroup(
            new SetShooterVelocity(m_shooter, 0, 0, true),
            new SetPivotAngle(m_pivot, 0, true))
          );
      default:
        return null;
    }
  }

  public SequentialCommandGroup shootToAmp(HandoffSequence m_handoff, CollectorArm m_collectorArm, Collector m_collector, Shooter m_shooter, Feeder m_feeder, Pivot m_pivot){
    return new SequentialCommandGroup(
      m_handoff.handoff(m_collector, m_collectorArm, m_feeder, m_pivot),
      new ParallelCommandGroup(
        new SetPivotAngle(m_pivot, 0, true),
        new SetShooterVelocity(m_shooter, 8, 8, true)
      ),
      new ParallelCommandGroup(
        new FeederShootCommand(m_feeder, 30),
        new ShooterWatchForNoteCommand(m_shooter)
      ),
      new ParallelCommandGroup(
        new SetShooterVelocity(m_shooter, 0, 0, true),
        new SetPivotAngle(m_pivot, 0, true)
      )
    );
  }
}