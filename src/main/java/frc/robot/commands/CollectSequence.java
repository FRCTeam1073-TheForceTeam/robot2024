// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorArm.POSE;


public class CollectSequence extends Command {
  /** Creates a new CollectShootCommand. */
  public CollectSequence() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SequentialCommandGroup collectNote(Collector m_collector, CollectorArm m_collectorArm) {
    switch(m_collectorArm.getPoseName()) {
      case START:
        return new SequentialCommandGroup(
          new CollectorIntakeCommand(m_collector, m_collectorArm),
          new ArmPoseCommand(m_collectorArm, POSE.STOW_INTERMEDIATE),
          new ArmPoseCommand(m_collectorArm, POSE.STOW)
        );
      case STOW:
        return new SequentialCommandGroup(
          new ArmPoseCommand(m_collectorArm, POSE.STOW_INTERMEDIATE),
          new ArmPoseCommand(m_collectorArm, POSE.START),
          new CollectorIntakeCommand(m_collector, m_collectorArm),
          new ArmPoseCommand(m_collectorArm, POSE.STOW_INTERMEDIATE),
          new ArmPoseCommand(m_collectorArm, POSE.STOW_INTERMEDIATE_2),
          new ArmPoseCommand(m_collectorArm, POSE.STOW)
        );
      case AMP:
        return new SequentialCommandGroup(
          new CollectorAmpCommand(m_collector, m_collectorArm)
        );
      default:
        return null;
    }
  }
}
