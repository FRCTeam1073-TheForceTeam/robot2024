// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.OI;

public class AlignToApriltag extends CommandBase {
  /** Creates a new AlignToGamePiece. */
  private Drivetrain drivetrain;
  private AprilTagFinder finder;
  private double maxVelocity;
  private double YTolerance;
  private double XTolerance;
  private double targetTagX;
  private double targetTagY;
  private double targetX;// current targetX
  private double targetY;//current target Y
  private double tagX;//current game piece position
  private double tagY;// current game piece position
  private OI oi;



  // State variables for execution:
  Boolean targetTag;
  double targetTagDistance;
  private ChassisSpeeds chassisSpeeds;
  int glitchCounter;
  double yOffset;

  
  


  public AlignToApriltag(Drivetrain drivetrain, AprilTagFinder finder, double maxVelocity,double XTolerance, double YTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.finder = finder;
    this.XTolerance = XTolerance;
    this.YTolerance = YTolerance;
    this.maxVelocity = maxVelocity;
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    targetTagX = 111; 
    targetTagY = 23; 
    addRequirements(drivetrain);
  }

  private double speedScaleY(double targetY){
    double scaleY = (360 - targetY) * 4.2e-5;
    return MathUtil.clamp(scaleY, 0, 0.01);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetX = targetTagX;
    targetY = targetTagY;
    System.out.println("Align To Game Piece Started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    finder.getClosestTag();

      tagX = finder.closestTagX;
      tagY = finder.closestTagY;
      tagX = MathUtil.clamp(tagX, 0, 10000);
      tagY = MathUtil.clamp(tagY, 0, 10000);
    


    double deltaX = targetX - tagX;
    double deltaY = targetY - tagY;
    double commandScaleY = speedScaleY(tagY);

    //ends command if game piece is close (x,y) to robot

    chassisSpeeds.vyMetersPerSecond = (deltaX) * commandScaleY;
    chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxVelocity, maxVelocity);

    chassisSpeeds.vxMetersPerSecond = (deltaY) * commandScaleY;
    chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, 0, maxVelocity);

    chassisSpeeds.omegaRadiansPerSecond = 0;

    drivetrain.setTargetChassisSpeeds(chassisSpeeds);


    SmartDashboard.putNumber("VY Meters pre second", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("VX Meters pre second", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Game Piece Max Velocity", maxVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.vxMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = 0;
    drivetrain.setTargetChassisSpeeds(chassisSpeeds);

    System.out.println("Align To april tag Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(finder.closestTag == -1){
      System.out.println("No April Tags to be found");
      return true;
    }

    if (Math.abs(targetX - tagX) < XTolerance && Math.abs(targetY - tagY) < YTolerance) {
      System.out.println("Aligned to Game Piece Finished");
      return true;
    }else{
      return false;
    }

  }
}