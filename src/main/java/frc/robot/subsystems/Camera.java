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
  private int camID;
  
  public Camera(SerialComms serialComms, int camID) {
    System.out.println("in camera constructor");
    this.serialComms = serialComms;
    System.out.println("camID in constructor");
    System.out.println(camID);
  }

  // public void getAprilTag(){
  //   System.out.println("in getAprilTag()");
  //   System.out.println("camID in getAprilTag()");
  //   System.out.println(camID);
  //   String cmdStr = "ti\n";
  //   String data = SerialComms.transact(cmdStr);
  //   System.out.println("gotAprilTag, data:");
  //   System.out.println(data);
  // }

  public void startRecordingTeleop(){
    serialComms.send("2,ti");
    System.out.println("printed ti");
  }

  public void startRecordingAutonomous(){
    serialComms.send("2,ai");
  }

  public void stopRecording(){
    serialComms.send("2,di");
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

