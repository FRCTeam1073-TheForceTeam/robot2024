// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class LaunchFeederToSpeaker extends SequentialCommandGroup{
    
  public LaunchFeederToSpeaker(){ //Shooter shooter, Feeder feeder, RangeFinder rangefinder

    }

  public SequentialCommandGroup runLaunchFeedertoSpeaker(Shooter m_shooter, Feeder m_feeder, Pivot m_pivot, RangeFinder m_rangeFinder){
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new InstantCommand(),
        new ParallelCommandGroup(
          new PivotRangeCommand(m_pivot, m_rangeFinder),
          new RunShooter(m_shooter, m_rangeFinder) //, m_rangefinder.getRange()),
        ),
        m_shooter::getCommandedToShoot
      ),
      new ParallelDeadlineGroup(
        new StopShooter(m_shooter),
        new RunFeeder(m_feeder, 30)
        //new WaitCommand(1),
      ),
      new SetPivotCommand(m_pivot, 0)
    );
  }

  public Command runDynamicAiming(Pivot m_Pivot, Shooter m_Shooter, RangeFinder m_RangeFinder, Drivetrain m_Drivetrain, AprilTagFinder m_AprilTagFinder){
    return new ParallelCommandGroup(
      new DynamicPivotRangeCommand(m_Pivot, m_RangeFinder, m_Drivetrain, m_AprilTagFinder),
      new DynamicRunShooter(m_Shooter, m_RangeFinder)
    );
  }
}