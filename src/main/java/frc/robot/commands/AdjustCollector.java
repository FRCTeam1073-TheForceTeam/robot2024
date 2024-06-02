// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class AdjustCollector extends Command {
  Collector collector;
  private double maxCollectRange = 0.33;
  private double minCollectRange = 0.32;
  private double prevRange = -1;
  private double currRange = -1;
  private boolean signSet = false;
  private int sign = 1;

  /** Creates a new AdjustCollector. */
  public AdjustCollector(Collector collector) {
    this.collector = collector;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(currRange != -1){
      prevRange = currRange;
    }
    currRange = collector.getRangeTOF1();
    if(currRange < minCollectRange){
      // if note is too close to ToF sensor, need to back out of the colector (slowly)
      if(signSet = false && prevRange != -1 && prevRange > currRange){
        signSet = true;
        sign *= -1;
      }
      collector.setTargetCollectorVelocity(sign * -0.7);
    }
    if(currRange > maxCollectRange){
      // if note is too far from the ToF sensor, pull farther into collector (slowly)
      if(signSet = false && prevRange != -1 && prevRange < currRange){
        signSet = true;
        sign *= -1;
      }
      collector.setTargetCollectorVelocity(sign * 0.7);
    }}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collector.setTargetCollectorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((collector.getRangeTOF1() <= maxCollectRange) && (collector.getRangeTOF1() >= minCollectRange));
  }
}
