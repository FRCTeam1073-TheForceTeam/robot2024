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
  private Camera camera;
  public SerialComms serialcomms;
  public boolean waiting;
  public int counter;
  public Map<String, Map<String, String>> tags = new HashMap<String, Map<String, String>>();
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

  public AprilTagFinder(Camera camera, SerialComms serialcomms) {
    this.serialcomms = serialcomms;
    this.camera = camera;
    this.waiting = false;
    this.counter = 0;
    this.tags = tags;
  }

  public void update(String[] info) {
    if (info.length < 3) {
      // System.out.println(String.format("Camera %s reporting no visible apriltags", info[0]));
      return;  // or just "return;" I forget
    }
    // if we do have tags, example:
    // args could be ["1", "ap", "2", "128", "178", "57", "45", "4", "127", "177", "56", "44"]
    // camera ID 1, apriltag command, two tags.
    // e.g. for tag id 2: xcenter, ycenter, tag area, and z-axis thing would be 128, 178, 57, 45.
    String cameraID = info[0];
    String cmd = info[1];
    for (int i=2; i<info.length; i+=5) {
      // start at 2 instead of 0. we don't care about args[0] and args[1] in here, so skip 'em.
      String tagID = info[i]; //on first iteration, args[2] is "2".
      // build an inner map for this tag's info
      Map<String, String> inner = new HashMap<String, String>();
      inner.put("xcenter", info[i+1]);
      inner.put("ycenter", info[i+2]);
      inner.put("area", info[i+3]);
      inner.put("zaxisorwhatever", info[i+4]);
      // this tag's items end at i+4. the next tag's ID is at i+5.
      // because we start the first iteration at 2, i == 7 at the top of the second iteration (2 + 5 = 7) and the tag ID is args[7].
      // if there were a third tag, its ID would be args[12] (7 + 5 = 12)

      // finally, add this tag's ID and info to the public tags map.
      this.tags.put(tagID, inner);
    }
  }

  @Override
  public void periodic() {
    if (this.counter == 25) {  // 25 periodic() iterations is roughly a half second. adjust to taste.
      // too long without an update. stop waiting, reset counter, clear map (too old to be useful)
      this.waiting = false;
      this.counter = 0;
      this.tags.clear();
    }

    if (this.waiting == false) {
        this.camera.requestAprilTags("0");  // 0 gets all visible tags
        this.waiting = true;
    }
    else {
      String[] info = this.camera.getAprilTagInfo();
      if (info.length > 0) { // double check an empty response is actually an empty String[]
        // we got a response. stop waiting and update.
        // note: ["1", "ap"] is a legit response, just means we don't see tags.
        this.update(info);
        this.waiting = false;
        this.counter = 0;
      }
      else {
        this.counter += 1;
      }
    }
  }
}
