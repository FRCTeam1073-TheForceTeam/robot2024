// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// // TODO: lots of Cole's apriltag/target code may live in here

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  //private SerialPort port = new SerialPort(1000000, SerialPort.Port.kUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);

  private SerialComms serialComms;
  private int camID;
  
  public Camera(SerialComms serialComms, int camID) {
    this.serialComms = serialComms;
    System.out.println("camID in constructor");
    System.out.println(camID);
    //public ArrayList<Byte> msg = new ArrayList();
  }

  public void buildCommand() {
  }

  public void getAprilTag(){
    System.out.println("in getAprilTag()");
    System.out.println("camID in getAprilTag()");
    System.out.println(camID);
    String cmdStr = "ti\n"; // NSARGENT: HARDCODED MSG HERE
    // a = autonomous init, t = teleop init, s = stop recording
    //byte[] cmdBytes = cmdStr.getBytes();
    String data = SerialComms.getVisionData(cmdStr);
    System.out.println("gotAprilTag, data:");
    System.out.println(data);
    //String dataAsASCII = new String(data, StandardCharsets.US_ASCII);
  }

  public void startRecordingTeleop(){
    serialComms.send("ti");
  }

  public void startRecordingAuto(){
    serialComms.send("ai");
  }

  public void stopRecording(){
    serialComms.send("di");
  }


  @Override
  public void periodic() {
    // System.out.println("getting apriltag data");
    // getAprilTag();
    try {
      Thread.sleep(1000);
    }
    catch (final InterruptedException e) {
      throw new RuntimeException(e);
    }
  }
}

