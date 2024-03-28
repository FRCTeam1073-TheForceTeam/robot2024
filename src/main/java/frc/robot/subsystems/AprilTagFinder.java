// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagFinder extends SubsystemBase {

  // Sensed target data type:
  public class TagData {
    public int id = -1;
    public int cx = 0;
    public int cy = 0;
    public int counterpartX_low = 0;
    public double counterpartX_high = 0;
    public double timestamp = 0;

    public boolean isValid() {
      if (id == -1) return false;
      else return true;
    }
  }
  public static int searchTagID = -1;  // use the SetSearchTagID to modify
  public static int searchTagID1 = -1;
  public int camID = 1;  // Camera ID to send to
  public byte outputBuffer[] = new byte[8];
  public TagData tagData = new TagData();
  private boolean aligned;


  }

  /// Sets the tagID we're searching for.
  public void setSearchTagId(int id, int id2) {
    this.searchTagID = id; // We're searching for this ID on next cycle.
    this.searchTagID1 = id2;
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

    }
  }

  @Override
  public void periodic() {


    // if -1, don't bother doing anything.
    if (this.searchTagID == -1) {
      this.tagData.id = -1;
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
          parseResponse(response);
          this.recvCounter += 1; // We received a packet.
          this.wait_counter = 0; // We're not waiting anymore.
          this.waiting_for_response = false;
        } else {
            // Else we don't yet have a full response.
            this.wait_counter += 1; // Count the cycles we're waiting.
        }
      }
    }

    aligned = Math.abs(tagData.cx - 160) <= 10;


    // Subsystem feedback:
    SmartDashboard.putNumber("AprilTag/DesiredID", this.searchTagID);

    // Found tag feedback:
    SmartDashboard.putNumber("AprilTag/ID", this.tagData.id);
    SmartDashboard.putBoolean("AprilTag/Valid", this.tagData.isValid()); // Allows dashboard indicator.
    SmartDashboard.putBoolean("AprilTag/Aligned", aligned);
  }

}
