// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagFinder extends SubsystemBase {
  /** Creates a new AprilTagFinder. */
  private Camera camera;
  public SerialComms serialcomms;
  public boolean waiting;
  public int counter;
  
  // map stuff might look something like this:
  // public Map<String, Map<String, String>> tags = new HashMap<String, Map<String, String>>();
  // tags = {
  //   '8': {'xcenter': 10, 'ycenter': 11},
  //   '2': {'xcenter': 5, 'ycenter': 7}
  // }

  public AprilTagFinder(Camera camera, SerialComms serialcomms) {
    this.serialcomms = serialcomms;
    this.camera = camera;
    this.waiting = false;
    this.counter = 0;
  }

  public void update(String sargeant) {
    SmartDashboard.putString("AprilTagInfo", sargeant);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.counter == 25) {
        this.counter = 0;
        this.waiting = false;
        // TODO: clear the map
        // this.map = {};
    }

    if (this.waiting == false) {
        camera.getAprilTagInfo("0");
        this.waiting = true;
    }
    else {
      String info = camera.receiveAprilTagInfo();
      if (info != "") {
        this.update(info);
      }
      else {
        this.counter += 1;
      }
    }
  }
}
