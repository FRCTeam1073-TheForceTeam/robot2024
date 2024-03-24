// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Headlight;

/*not in use right now */

public class TurnOnHeadlight extends Command {
    private Headlight headlight;
  /** Creates a new TurnOnHeadlight. */


  public TurnOnHeadlight(Headlight headlight, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.headlight = headlight;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    headlight.setHeadlight(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    headlight.setHeadlight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
