// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: figure out when or how to clear tags. how long before they should they age out?
//       note: periodic() wipes the map after ~0.5s without the camera returning any tags, whether it replies with no tags or stops replying entirely.
public class AprilTagFinder extends SubsystemBase {

  public class TagData {
    public int id = -1;
    public int cx = 0;
    public int cy = 0;
    public int area = 0;
  }


  public SerialComms serialcomms;
  public boolean waiting_for_response = false;
  public int wait_counter = 0;
  public int tagID = 4;  // use the SetAprilTagID to modify
  public byte output_buffer[] = new byte[8];
  public TagData tag_data = new TagData();
  public int send_counter = 0;
  public int recv_counter = 0;

  // public Map<int , TagaData> tags = new HashMap<String, Map<String, String>>();

  // https://docs.oracle.com/javase/8/docs/api/java/util/HashMap.html
  // map example. given:
  //   1. we're looking at april tags 2 and 4
  //   2. we care about four stats for each tag (easy to adjust later)
  //
  // tags = {
  //   "2": {"xcenter": "128", "ycenter": "178", "area": "57", "zaxisorwhatever": "45"},
  //   "4": {"xcenter": "127", "ycenter": "177", "area": "56", "zaxisorwhatever": "44"}
  // }
  // example .get()'s:
  // tags.get("2") == {"xcenter": "128", "ycenter": "178", "area": "57", "zaxisorwhatever": "45"}
  // tags.get("9") == null
  // caller can deal with null or call tags.containsKey("9") first.

  public AprilTagFinder(SerialComms serialcomms) {
    this.serialcomms = serialcomms;
    this.waiting_for_response = false;
    this.wait_counter = 0;
    this.tag_data.id = -1;
  }


  /// Sets the tagID we're searching for.
  public void setTagId(int id) {
    this.tagID = id; // We're searching for this ID on next cycle.
  }

  /// Returns the current tag data found.
  public TagData getTagData() {
    return tag_data;
  }


  // TODO: add System.currentTimeMillis() to inner maps, age out existing entries (do age outs before puts)
  public void parseResponse(byte[] response) {

    if (response.length < 8) {
      System.out.println(String.format("Invalid camera response length: %d", response.length));
      this.tag_data.id = -1;
      return;
    }

    tag_data.id = response[2]; // ID of the found tag.
    tag_data.cx = response[3] * 4; // Undo packing so it fits a byte.
    tag_data.cy = response[4] * 4; // Undo packing so it fits a byte.
    tag_data.area = response[5] * 64; // Undo packing so it fits a byte.
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AprilTagFinderDesiredID", this.tagID);
    // if -1, don't bother
    if (this.tagID == -1) {
      return;
    }

    if (this.waiting_for_response == true && this.wait_counter >= 25) {  // 25 periodic() iterations is roughly a half second. adjust to taste.
      // too long without an update. stop waiting, reset counter, clear map (too old to be useful)
      this.waiting_for_response = false;
      this.wait_counter = 0;
      SmartDashboard.putString("AprilTagFinderTimedout", String.format("hit counter %s", System.currentTimeMillis()));
      // Our tag data is too old, clear it:
      this.tag_data.id = -1;
      return;
    }

    if (this.waiting_for_response == false) {
        // this.camera.requestAprilTags("0");  // 0 gets all visible tags
        output_buffer[0] = 1;
        output_buffer[1] = 3; // Request april tags.
        output_buffer[2] = (byte) this.tagID; // Request specific tag ID.

        // Send request to camerea:
        this.serialcomms.send(output_buffer);
        this.send_counter +=1;
       

        this.waiting_for_response = true;
        this.wait_counter = 0;
    }
    else {
      // We are waiting for a response:
      var response = this.serialcomms.receive();

      if (response != null) {
        parseResponse(response);
        this.recv_counter += 1;
        this.wait_counter = 0;
        this.waiting_for_response = false;
      } else {
          // Else we don't yet have a full response.
          this.wait_counter += 1; // We've waited a whole cycle...
      }
    }

    SmartDashboard.putNumber("AprilTag SendCounter",send_counter);
    SmartDashboard.putNumber("AprilTag RecvCounter",recv_counter);

    SmartDashboard.putNumber("AprilTagID", this.tag_data.id);
    SmartDashboard.putNumber("ApriltTag X", this.tag_data.cx);
    SmartDashboard.putNumber("AprilTag Y", this.tag_data.cy);
    SmartDashboard.putNumber("AprilTag Area", this.tag_data.area);
  }

}
