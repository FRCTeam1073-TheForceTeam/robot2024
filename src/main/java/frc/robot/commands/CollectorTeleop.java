// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class CollectorTeleop extends Command {
  /** Creates a new CollectorTeleop. */
  Collector m_collector;
  Drivetrain m_drivetrain;
  OI m_OI;
  double minRange;
  double maxRange;


  public CollectorTeleop(Collector collector, Drivetrain ds, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collector;
    m_drivetrain = ds;
    m_OI = oi;
    minRange = 0.4;
    maxRange = 0.72;

    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(m_OI.getOperatorRawButton(5))
    {
      m_collector.setTargetCollectorVelocity(3); //meters per sec
      if(m_collector.getRangeTOF() > maxRange){
        m_collector.setTargetCollectorVelocity(0);
      }
    }
    else if(m_OI.getOperatorRawButton(6)) //intake
    {
      double vel = 1.5 * (m_drivetrain.getChassisSpeeds().vxMetersPerSecond + 1);
      m_collector.setTargetCollectorVelocity(-vel); //meters per sec      
      if(m_collector.getRangeTOF() < minRange){
        m_collector.setTargetCollectorVelocity(0);
      }
    }
    else {
      m_collector.setTargetCollectorVelocity(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setTargetCollectorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
