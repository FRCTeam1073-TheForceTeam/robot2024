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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SerialComms;

import java.util.ArrayList;

public class AprilTagFinder extends SubsystemBase {
  /** Creates a new GamePieceFinder. */
  public class Target {
    public String type;
    public int ID = 0;
    public double rotation = 0.0;
    public double centerX = 0.0;
    public double centerY = 0.0;
    public double centerH = 0.0;
    public double centerW = 0.0;
  }

  private NetworkTable networkTable;
  private NetworkTableEntry cubePieceEntry;
  private NetworkTableEntry conePieceEntry;
  public String tableName;
  public double closestTagArea;
  public int closestTag;
  private Drivetrain driveSubsystem;
  private Transform3d cameraTransform;
  private GamePiecePoseEstimator poseEstimator;
  public double closestTagX;
  public double closestTagY;
  public ArrayList<Target> targets;

  

  public class GamePiecePoseEstimator {
    //public Transform3d estimate(){
    //}
  }

  public AprilTagFinder(Drivetrain ds) {
    driveSubsystem = ds;
    this.tableName = tableName;

  }

  @Override
  public void periodic() {
    Number[] tagDataArray = cubePieceEntry.getNumberArray(new Number[0]);
    int numTag = tagDataArray.length/7;
    // Reset search variables for clostest to empty:

    for (int i = 0; i < numTag; i = i +1){
      targets.clear();
      Target tagData = new Target();

      tagData.centerX = tagDataArray[i*7 + 0].doubleValue();
      tagData.centerY = tagDataArray[i*7 + 1].doubleValue();
      tagData.centerW = tagDataArray[i*7 + 2].doubleValue();
      tagData.centerH = tagDataArray[i*7 + 3].doubleValue();

      targets.add(tagData);

      double area = tagData.centerW * tagData.centerH;
      
      if (area > closestTagArea) {
        closestTagArea = area;
        closestTag = i;
      }

    }

    if (closestTag > -1) {
      closestTagX = (targets.get(closestTag).centerX) + (targets.get(closestTag).centerW/2);
      closestTagY = (targets.get(closestTag).centerY) + (targets.get(closestTag).centerH/2);
    }



    // if (!closestGamePiece.equals("None")){
    //   SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), closestGamePiece);
    // }else{
    //   SmartDashboard.putString(String.format("%s/ClosestGamePiece", tableName), closestGamePiece);
    // }
    SmartDashboard.putNumber("closestTag", closestTag);
    SmartDashboard.putNumber("closestTagX", closestTagX);
    SmartDashboard.putNumber("closestTagY", closestTagY);
    SmartDashboard.putNumber("closestTagArea", closestTagArea);
  }


  public Pose2d getFieldPoseCube() {
    if (targets.size() > 0) {
      //TODO: Implement this method when needed
      return null; // Until we implement it.
    }else{
      return null;
    }
  }


  public int getClosestTag() {
    return closestTag;
  }

}