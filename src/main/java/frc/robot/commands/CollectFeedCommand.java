// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CollectorArm.POSE;


public class CollectFeedCommand extends Command 
{
  private CollectorArm m_collectorArm;

  /** Creates a new CollectShootCommand. */
  public CollectFeedCommand(CollectorArm m_collectorArm) {
    this.m_collectorArm = m_collectorArm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public boolean isStowed(){
    return m_collectorArm.getPoseName() == POSE.STOW;
  }

  public SequentialCommandGroup runCollectFeedCommand(Drivetrain m_drivetrain, Collector m_collector, CollectorArm m_collectorArm, Pivot m_pivot, Feeder m_feeder, Shooter m_shooter) {
    ArmPoseTeleop armCommands = new ArmPoseTeleop(m_collectorArm);
    return new SequentialCommandGroup(
      new ConditionalCommand(
        new SetPivotCommand(m_pivot, -0.6), //On true
        new ParallelCommandGroup( //on False
          armCommands.stowPose(),
          new SetPivotCommand(m_pivot, -0.6)
        ), 
        this::isStowed), //checks if arm is already stowed. If it is, then don't go to stow again
      new ParallelDeadlineGroup(
        new LoadFeeder(m_feeder, 1.5),
        new CollectorIntakeOutCommand(m_collector, m_collectorArm, m_drivetrain)
      ),
      new ParallelCommandGroup(
        new ArmPoseCommand(m_collectorArm, POSE.START),
        new ParallelDeadlineGroup(
          new WaitCommand(0.25),
          new AdjustFeed(m_feeder)
        )
      ),
      new SetShooterVel(m_shooter, 20, 20, false)
    );
  }

  public SequentialCommandGroup runCollectCommand(Drivetrain m_drivetrain, Collector m_collector, CollectorArm m_collectorArm){
    ArmPoseTeleop armCommands = new ArmPoseTeleop(m_collectorArm);

    return new SequentialCommandGroup(
      new CollectorIntakeCommand(m_collector, m_collectorArm, m_drivetrain),
      armCommands.stowPose()
    );
  }
}
