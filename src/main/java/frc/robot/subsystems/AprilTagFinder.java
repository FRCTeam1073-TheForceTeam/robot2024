// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SerialComms;

import java.util.ArrayList;

public class AprilTagFinder extends SubsystemBase {
  /** Creates a new GamePieceFinder. */
  public class Target {
    // ID, X_translation, Y_Transtlation, and Z_translation
    // Z_translation = far away
    // Y_translation = height
    // TODO x_translation = rotation???
    public String type;
    public static int ID = 0;
    public static double X_translation = 0.0;
    public static double Y_translation = 0.0;
    public static double Z_translation = 0.0;
  }

  public int closestTag;
  private Drivetrain driveSubsystem;
  private Transform3d cameraTransform;
  private GamePiecePoseEstimator poseEstimator;
  public ArrayList<Target> targets;

  public class GamePiecePoseEstimator {
    // public Transform3d estimate(){
    // }
  }

  public AprilTagFinder(Drivetrain ds) {
    driveSubsystem = ds;
   

  }

  @Override
  public void periodic() {
    Number[] tagDataArray = cubePieceEntry.getNumberArray(new Number[0]);
    int numTag = tagDataArray.length / 4;
    // Reset search variables for clostest to empty:

    for (int i = 0; i < numTag; i = i + 1) {
      targets.clear();
      Target tagData = new Target();

      tagData.ID = tagDataArray[i * 4 + 0].intValue();
      tagData.X_translation = tagDataArray[i * 4 + 1].doubleValue();
      tagData.Y_translation = tagDataArray[i * 4 + 2].doubleValue();
      tagData.Z_translation = tagDataArray[i * 4 + 3].doubleValue();
      targets.add(tagData);

    }

    // if (!closestGamePiece.equals("None")){
    // SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName),
    // closestGamePiece);
    // }else{
    // SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName),
    // closestGamePiece);
    // }
    SmartDashboard.putNumber("ID", Target.ID);
    SmartDashboard.putNumber("X_translation", Target.X_translation);
    SmartDashboard.putNumber("Y_translation", Target.Y_translation);
    SmartDashboard.putNumber("Z_translation", Target.Z_translation);
  }

}
