// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

/** Add your docs here. */
public class LaunchFeederToSpeaker extends SequentialCommandGroup{
    
  public LaunchFeederToSpeaker(){ //Shooter shooter, Feeder feeder, RangeFinder rangefinder

    }

  public SequentialCommandGroup runLaunchFeedertoSpeaker(Shooter m_shooter, Feeder m_feeder){
    return new SequentialCommandGroup(
      new RunShooter(m_shooter, 7.7), //, m_rangefinder.getRange()),
     new ParallelRaceGroup(
       new RunFeeder(m_feeder, 30), 
       new WaitCommand(1)),
      new StopShooter(m_shooter),
      new RunFeeder(m_feeder, 0));
    }
}