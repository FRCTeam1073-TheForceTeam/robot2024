/* run motor until note hits beam break */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trigger;

public class LoadTrigger extends Command {
  private Shooter shooter;
  private Trigger trigger;
  /** Creates a new LoadTrigger. */
  public LoadTrigger(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trigger.setTopTriggerMotorVelocity(5);
    trigger.setBottomTriggerMotorVelocity(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trigger.setTopTriggerMotorVelocity(0);
    trigger.setBottomTriggerMotorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trigger.noteIsInTrigger();
  }
}
