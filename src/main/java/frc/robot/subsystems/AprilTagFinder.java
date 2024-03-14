// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: figure out when or how to clear tags. how long before they should they age out?
//       note: periodic() wipes the map after ~0.5s without the camera returning any tags, whether it replies with no tags or stops replying entirely.
public class AprilTagFinder extends SubsystemBase {

  // April tag data
  public class TagData {
    public int id = -1;
    public int cx = 0;
    public int cy = 0;
    public int area = 0;
    public double timestamp = 0;
  }

  public SerialComms serialcomms;
  public boolean waiting_for_response = false;
  public int wait_counter = 0;
  public int tagID = 10;  // use the SetAprilTagID to modify
  public int camID = 1;  // Camera ID to send to
  public byte outputBuffer[] = new byte[8];
  public TagData tagData = new TagData();
  public int sendCounter = 0;
  public int recvCounter = 0;

  public AprilTagFinder(SerialComms serialcomms) {
    this.serialcomms = serialcomms;
    this.waiting_for_response = false;
    this.wait_counter = 0;
    this.tagData.id = -1;
    for (int idx=0;idx < this.outputBuffer.length; ++idx) {
      this.outputBuffer[idx] = 0;
    }
  }


  /// Sets the tagID we're searching for.
  public void setTagId(int id) {
    this.tagID = id; // We're searching for this ID on next cycle.
  }

  /// Returns the current tag data found.
  public TagData getTagData() {
    return tagData;
  }


  public void parseResponse(byte[] response) {
    if (response.length < 8) {
      System.out.println(String.format("Invalid camera response length: %d", response.length));
      this.tagData.id = -1;
      return;
    }

    tagData.id = response[2]; // ID of the found tag.
    //tagData.cx = response[3] * 2; // Undo packing so it fits a byte.
    tagData.cx = (response[3] & 0xFF) * 2; // Undo packing so it fits a byte.
    //tagData.cy = response[4] * 2; // Undo packing so it fits a byte.
    //tagData.cy = Byte.toUnsignedInt(response[4]) * 2;
    tagData.cy = (response[4] & 0xFF) * 2;
    //tagData.area = response[5] * 64; // Undo packing so it fits a byte.
    tagData.area = (response[5] & 0xFF) * 64;
    tagData.timestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("AprilTagFinderDesiredID", this.tagID);
    // if -1, don't bother doing anything.
    if (this.tagID == -1) {
      this.tagData.id = -1;
    } else if (this.waiting_for_response == false) {
       // We're not waiting for an answer, so send a new request.
        outputBuffer[0] = (byte) (this.camID & 0xFF); // Camera ID.
        outputBuffer[1] = 0x03; // Command: Find april tag = 3
        outputBuffer[2] = (byte) (this.tagID & 0xFF); // Request specific tag ID.

        // Send request to camerea:
        SmartDashboard.putRaw("SerialCommsSendRaw", outputBuffer);
        this.serialcomms.send(outputBuffer);
        this.sendCounter +=1; // We sent a packet.
       
        // We're now waiting for a response.
        this.waiting_for_response = true;
        this.wait_counter = 0;
    } else { 
      // waiting_for_response = true...

       // Too long without an update. stop waiting, reset counter, clear tag data (too old to be useful)
      if (this.wait_counter >= 25) {
        this.waiting_for_response = false;
        this.wait_counter = 0;
        SmartDashboard.putString("AprilTagFinderTimedout", String.format("hit counter %s", System.currentTimeMillis()));
        // Our tag data is too old, clear it:
        this.tagData.id = -1;
      } else {
        // We are waiting for a response:
        var response = this.serialcomms.receive();
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

    SmartDashboard.putNumber("AprilTag SendCounter", sendCounter);
    SmartDashboard.putNumber("AprilTag RecvCounter", recvCounter);

    SmartDashboard.putNumber("AprilTag ID", this.tagData.id);
    SmartDashboard.putNumber("AprilTag X", this.tagData.cx);
    SmartDashboard.putNumber("AprilTag Y", this.tagData.cy);
    SmartDashboard.putNumber("AprilTag Area", this.tagData.area);
  }

}
