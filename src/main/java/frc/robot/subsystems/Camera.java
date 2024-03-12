// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  private SerialComms serialComms;
  private String camID;
  
  public Camera(SerialComms serialComms, String camID) {
    System.out.println("in camera constructor");
    this.serialComms = serialComms;
    this.camID = camID;
    System.out.println(String.format("camID: %s", camID));
  }


  
  public void requestAprilTags() {
      byte[] rawbytes = {0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
      this.serialComms.send(rawbytes);
  }

  public String[] getAprilTagInfo() {
    String msgString = serialComms.receive();
    SmartDashboard.putString("CameraIncomingMsg", msgString);
  // we don't do this in SerialComms because other subsystems might not use the comma-separated system we do
    String[] msgStringArray = msgString.split(",");
    return msgStringArray;
  }

  @Override
  public void periodic() {
  }
}

