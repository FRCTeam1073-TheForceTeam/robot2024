// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AprilTagFinder extends SubsystemBase {

  // Sensed target data type:
  public class TagData {
    public int id = -1;
    public int x = 0;
    public int y = 0;
    public double timestamp = 0;

    public boolean isValid() {
      if (id == -1) return false;
      else return true;
    }
  }
  //TODO: replace code to pull smartdashboard and replace with getLatestData() and hasTarget() methods
  
  // Communications:
  public String tableName;
  //public PhotonCamera camera = new PhotonCamera("photonvision");
  public static int searchTagID = -1;  // use the SetSearchTagID to modify
  TagData tagData = new TagData();
  public int wait_counter = 0;
  private boolean aligned;
  public boolean waiting_for_response;
  public PhotonCamera camera = new PhotonCamera("Global_Shutter_Camera"); 



  public AprilTagFinder() {
    this.tableName = tableName;
  }

  /// Sets the tagID we're searching for.
  public void setSearchTagId(int id) {
  }

  /// Returns the current tag data found.
  public TagData getCurrentTagData() {
    return tagData;
  }

  public boolean tagFound() {
    return this.tagData.isValid();
  }

  public boolean isAligned(){
    return aligned;
  }

  public TagData readTagData(){
    PhotonPipelineResult response = camera.getLatestResult();
    if(response.hasTargets() == true){
      tagData.id = response.getFiducialId();
      tagData.x = (int) SmartDashboard.getNumber("targetPixelX", -1);
      tagData.y = (int) SmartDashboard.getNumber("targetPixelsY", -1);
      tagData.timestamp = Timer.getFPGATimestamp();
    }
    return tagData;
  }
  
    //tagData.timestamp = Timer.getFPGATimestamp();

  @Override
  public void periodic() {


    // if -1, don't bother doing anything.
    if (this.searchTagID == -1) {
      this.tagData.id = -1;
    } 
       // Too long without an update. stop waiting, reset counter, clear tag data (too old to be useful)
      if (this.wait_counter >= 25) {
        this.waiting_for_response = false;
        this.wait_counter = 0;
        SmartDashboard.putString("AprilTag/Status", String.format("Timeout at %d", System.currentTimeMillis()));
        // Our tag data is too old, clear it:
        this.tagData.id = -1;
      } else {
        // We are waiting for a response:
        if (response != null) {
          //this.wait_counter = 0; // We're not waiting anymore.
          //this.waiting_for_response = false;
          assert true;
        } else {
            // Else we don't yet have a full response.
            this.wait_counter += 1; // Count the cycles we're waiting.
        }
     // }
    //}

    aligned = Math.abs(tagData.x - 160) <= 10;


    // Subsystem feedback:
    SmartDashboard.putNumber("AprilTag/DesiredID", this.searchTagID);

    // Found tag feedback:
    SmartDashboard.putNumber("AprilTag/ID", this.tagData.id);
    SmartDashboard.putNumber("AprilTag/X(Added)", this.tagData.x);
    SmartDashboard.putNumber("AprilTag/X", tagData.x);
    SmartDashboard.putNumber("AprilTag/Y", this.tagData.y);
    SmartDashboard.putBoolean("AprilTag/Valid", this.tagData.isValid()); // Allows dashboard indicator.
    SmartDashboard.putBoolean("AprilTag/Aligned", aligned);
  }

}
}
