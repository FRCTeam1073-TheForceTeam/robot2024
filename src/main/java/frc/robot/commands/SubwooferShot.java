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
public class SubwooferShot extends SequentialCommandGroup{
    
  public SubwooferShot(){ //Shooter shooter, Feeder feeder, RangeFinder rangefinder

    }

  public SequentialCommandGroup runSubwooferShot(Shooter m_shooter, Feeder m_feeder, Pivot m_pivot){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SetPivotCommand(m_pivot, 0.0),
        new SetShooterVel(m_shooter, 25, 25) //, m_rangefinder.getRange()),
      ),
      new ParallelCommandGroup(
        new RunFeeder(m_feeder, 30),
        new StopShooter(m_shooter)
      ),
      new SetPivotCommand(m_pivot, 0));
    }
}