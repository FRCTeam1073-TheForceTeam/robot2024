// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AmpShootCommand extends Command {
  /** Creates a new AmpShot. */
  public AmpShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

  }
  public SequentialCommandGroup ampShot(Shooter m_shooter, Feeder m_feeder, Pivot m_pivot) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SetPivotCommand(m_pivot, 0),
        new SetShooterVel(m_shooter, 8, 8) //, m_rangefinder.getRange()),
      ),
      //new WaitCommand(1),
      new ParallelCommandGroup(
        new RunFeeder(m_feeder, 30),
        //new WaitCommand(1),
        new StopShooter(m_shooter)
      ),
      new SetPivotCommand(m_pivot, 0));
    }
  }