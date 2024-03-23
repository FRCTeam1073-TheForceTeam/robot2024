// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorArm.POSE;

public class ArmPoseCommand extends Command {

  CollectorArm m_arm;
  POSE m_pose;

  double m_targetLift;
  double m_targetExtend;
  double m_liftTolerence = 0.015;
  double m_extendTolerence = 0.005;

  boolean m_extendFlag;

  /** Creates a new ArmPoseCommand. 
   * @param extendInterpolateFlag is set to true when you want to use the interpolator for extend
  */
  public ArmPoseCommand(CollectorArm arm, POSE pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_pose = pose;
    //m_extendFlag = extendInterpolateFlag;
    addRequirements(arm);
  }

  public ArmPoseCommand(CollectorArm arm, POSE pose, double liftTolerence, double extendTolerence){
    m_arm = arm;
    m_pose = pose;

    m_liftTolerence = liftTolerence;
    m_extendTolerence = extendTolerence;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_pose) {
      case START:
        m_targetLift = 0.0;
        m_targetExtend = 0.0;
        break;
      case STOW_INTERMEDIATE_1:
        // m_targetLift = 0.25;
        m_targetLift = 0.5;
        m_targetExtend = 0.0;
        break;
      case STOW_INTERMEDIATE_2:
        m_targetLift = 0.7;
        m_targetExtend = 0.05;
        break;
      case STOW_INTERMEDIATE_3:
        m_targetLift = 0.27;
        m_targetExtend = 0.06;
        break;
      case STOW_INTERMEDIATE_4:
        m_targetLift = 0.26;
        m_targetExtend = 0.1;
        break;
      case STOW:
        m_targetLift = 0.27;
        m_targetExtend = 0.101;
        break;
      case HANDOFF:
        m_targetLift = 0.35;
        m_targetExtend = 0.108;
        break;
      case AMP:
        m_targetLift = 1.9453125;
        m_targetExtend = 0.05;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setTargetLiftAngle(m_targetLift);
    // if(!m_extendFlag){
    m_arm.setTargetExtendLength(m_targetExtend);
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setPoseName(m_pose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double liftError = Math.abs(m_arm.getCurrentLiftAngle() - m_targetLift);
    double extendError = Math.abs(m_arm.getCurrentExtendLength() - m_targetExtend);
    if((liftError < m_liftTolerence) && (extendError < m_extendTolerence)){
      return true;
    }
    else{
      return false;
    }
  }
}
