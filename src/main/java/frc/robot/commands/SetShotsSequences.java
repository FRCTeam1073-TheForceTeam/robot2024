// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class SetShotsSequences extends Command {
  /** Creates a new SetShots. */
  public SetShotsSequences() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ParallelCommandGroup runSubwooferShot(Pivot m_pivot, Shooter m_shooter){
    return new ParallelCommandGroup(
      new SetPivotCommand(m_pivot, 0.0),
      new SetShooterVel(m_shooter, 20, 20, true)
    );
  }

  public ParallelCommandGroup runPodiumShot(Pivot m_pivot, Shooter m_shooter){
    return new ParallelCommandGroup(
      new SetPivotCommand(m_pivot, -0.58),
      new SetShooterVel(m_shooter, 24, 24, true)
    );
  }

  public ParallelCommandGroup runFarShot(Pivot m_pivot, Shooter m_shooter){
    return new ParallelCommandGroup(
      new SetPivotCommand(m_pivot, -0.7415),
      new SetShooterVel(m_shooter, 32, 32, true)
    );
  }
}