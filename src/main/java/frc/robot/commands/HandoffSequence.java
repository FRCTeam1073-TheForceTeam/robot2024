// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CollectorArm.POSE;

public class HandoffSequence extends Command {
  /** Creates a new HandoffSequence. */
  public HandoffSequence() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SequentialCommandGroup handoff(Collector m_collector, CollectorArm m_collectorArm, Feeder m_feeder, Pivot m_pivot) {
    SequentialCommandGroup handoff = new SequentialCommandGroup(
      new SetPivotAngle(m_pivot, -0.6, true),
      new ParallelDeadlineGroup(
        new FeederLoadCommand(m_feeder, 1.5),
        new CollectorHandoffCommand(m_collector, m_collectorArm)
      ),
      new ParallelCommandGroup(
        new ArmPoseCommand(m_collectorArm, POSE.START),
        new FeederAdjustCommand(m_feeder)
      )
    );
    
    // If the note is not in the feeder, hands off the note and prepares to shoot
    if(!m_feeder.noteIsInFeeder()){
      switch(m_collectorArm.getPoseName()) {
        case STOW:
          return handoff;
        case START:
          return new SequentialCommandGroup(
            new ArmPoseCommand(m_collectorArm, POSE.STOW),
            handoff
          );
        default:
          return null;
      }
    }
    // If the note was already in the feeder, just prepares to shoot
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ArmPoseCommand(m_collectorArm, POSE.START),
        new FeederAdjustCommand(m_feeder)
      )
    );
  }
}
