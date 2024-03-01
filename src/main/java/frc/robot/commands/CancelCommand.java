// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CollectorArm.POSE;

public class CancelCommand extends Command {
  /** Creates a new CancelCommand. */
  public CancelCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SequentialCommandGroup cancel(Collector m_collector, CollectorArm m_collectorArm, Shooter m_shooter, Feeder m_feeder, Pivot m_pivot) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new CollectorSpeedCommand(m_collector, 0),
        new ArmPoseCommand(m_collectorArm, POSE.START),
        new StopShooter(m_shooter),
        new StopFeeder(m_feeder),
        new SetPivotCommand(m_pivot, 0)
      )
    );
  }
}

  