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
    // String cmdStr = "%s,a\n".formatted(camID); // NSARGENT: HARDCODED MSG HERE
    String cmdStr = "123abc\njkl\n"; // NSARGENT: HARDCODED MSG HERE
    //byte[] cmdBytes = cmdStr.getBytes();
    String data = SerialComms.getVisionData(cmdStr);
    System.out.println("gotAprilTag, data:");
    System.out.println(data);
    //String dataAsASCII = new String(data, StandardCharsets.US_ASCII);
  }


  // public void sendAprilTag() {
  //   byte[] cmdBytes = "1,a\n".getBytes();
  //   int bytesWritten = port.write(cmdBytes, cmdBytes.length);
  // }


  // NSARGENT: getAprilTag() looks better
  // public ArrayList<Byte> getMsg()
  // {
  //   ArrayList<Byte> msg = new ArrayList<Byte>();
  //   byte[] oneByte = serialComms.getVisionData();
  //   if (oneByte == "\n".getBytes()) {
  //     return msg;
  //   }
  //   else {
  //     byte oneActualByte = oneByte[0];
  //     msg.add(oneActualByte);
  //   }
  // }


  @Override
  public void periodic() {
    //int bytesWaiting = port.getBytesReceived();  // returns the number of bytes waiting to be read, without actually reading them
    // byte[] cmdBytes = "2,g,0,1,2,3,4,5,6,7,8,9\n".getBytes("ASCII");
    //int cmdBytesLen = cmdBytes.length;
    //Integer wrote = port.write(cmdBytes, cmdBytesLen); // second arg is maximum bytes to write, which isn't a big deal for us
    //System.out.println(String.format("just wrote this many bytes: %d", wrote));
    // ArrayList<Byte> aprilTag = getAprilTag()
    System.out.println("getting apriltag data");
    getAprilTag();
    try {
      Thread.sleep(1000);
    }
    catch (final InterruptedException e) {
      throw new RuntimeException(e);
    }
    //ArrayList<Byte> msg = getMsg();
    //this.msg = getMsg();
    //String msgString = msg.toString();
    // char cmdChar = (char)msg[2];
    // if (cmdChar == 'a') {
    //   //do whatever apriltags do
  }
}

