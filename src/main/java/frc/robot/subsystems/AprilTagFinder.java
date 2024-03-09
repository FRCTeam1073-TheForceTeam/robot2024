// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

public class AprilTagFinder extends DiagnosticsSubsystem {
  /** Creates a new AprilTagFinder. */
  private Camera camera;
  public int tagID = 0;
  public double centerX = 0.0;
  public double centerY = 0.0;
  public double area = 0.0;
  public Map<String, Map<String, String>> tags = new HashMap<String, Map<String, String>>();

  // tags = {
  //   '8': {'xcenter': 10, 'ycenter': 11},
  //   '2': {'xcenter': 5, 'ycenter': 7}
  // }

  public AprilTagFinder(Camera camera) {
    tagID = 0;
    centerX = 0.0;
    centerY = 0.0;
    area = 0.0;
  }
  
  public void PublishToSmartDashboard() {
  }

  public void getAprilTagInfo(String tagid) { //unless it's an int
    //camera stuff
    String visibleTags = camera.getAprilTagInfo("0");
    //1,ap,1,2,3,4,5,11,12,13,14,15
    // tagid, xcenter, ycenter, area, z
    String[] args = visibleTags.split(",");
    for(int i=0; i < args.length; i+=5) {  // presuming there's six vals per tag, figure it out later
      //tags.put("tagid", visibleTagsButAList[i+0]);
      String tagID = args[i];
      Map<String, String> tagStats = new HashMap<String, String>();
      tagStats.put("xcenter", args[i+1]);
      tagStats.put("xcenter", args[i+2]);
      // and so on
      tags.put(tagID, tagStats);

      }
      tags.put(tagid, stats);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
