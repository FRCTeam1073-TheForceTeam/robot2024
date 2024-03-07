// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class LaunchFeederToSpeakerSequence extends SequentialCommandGroup{
    
  public LaunchFeederToSpeakerSequence(){ //Shooter shooter, Feeder feeder, RangeFinder rangefinder

    }

  public SequentialCommandGroup runLaunchFeedertoSpeaker(Shooter m_shooter, Feeder m_feeder, Pivot m_pivot, RangeFinder m_rangeFinder){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SetPivotRange(m_pivot, m_rangeFinder, true),
        new ShootAtRange(m_shooter, m_rangeFinder) //, m_rangefinder.getRange()),
      ),
      //new WaitCommand(1),
      new ParallelCommandGroup(
        new FeederShootCommand(m_feeder, 30),
        //new WaitCommand(1),
        new WatchShooterForNote(m_shooter)
      ),
      new ParallelCommandGroup(
        new SetShooterVelocity(m_shooter, 0, 0, true),
        new SetPivotAngle(m_pivot, 0, true))
      );
    }
}