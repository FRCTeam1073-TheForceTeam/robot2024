// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;

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

  public void startRecordingTeleop(){
    serialComms.send(this.camID + ",ti");
  }

  public void startRecordingAutonomous(){
    serialComms.send(this.camID + ",ai");
  }

  public void stopRecording(){
    serialComms.send(this.camID + ",di");
  }
  public String getAprilTagInfo(String tagID){
    serialComms.send(this.camID + ",ap," + tagID);
    return serialComms.receive();
  }

  @Override
  public void periodic() {
    // System.out.println("getting apriltag data");
    // getAprilTag();
    // try {
    //   Thread.sleep(1000);
    // }
    // catch (final InterruptedException e) {
    //   throw new RuntimeException(e);
    // }
  }
}

