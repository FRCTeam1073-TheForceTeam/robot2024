// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: lots of Cole's apriltag/target code may live in here

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  //private SerialPort port = new SerialPort(1000000, SerialPort.Port.kUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);

  private SerialComms serialComms;
  private int camID;
  
  public Camera(SerialComms serialComms, int camID) {
    this.serialComms = serialComms;
    //public ArrayList<Byte> msg = new ArrayList();
    if(serialComms == null){
      System.out.println(serialComms.toString());
    }
    else{
      
        System.out.println(serialComms.toString());

    }
  }

  public void buildCommand() {
  }

  public void getAprilTag(){
    String cmdStr = "%s,a\n".formatted(camID);
    byte[] cmdBytes = cmdStr.getBytes();
    ArrayList<Byte> data = serialComms.getVisionData(cmdBytes);
    System.out.println(data);
  }


  // public void sendAprilTag() {
  //   byte[] cmdBytes = "1,a\n".getBytes();
  //   int bytesWritten = port.write(cmdBytes, cmdBytes.length);
  // }

  // public ArrayList<Byte> getMsg()
  // {
  //   ArrayList<Byte> msg = new ArrayList();
  //   byte[] oneByte = serialComms.getMsg(1);
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

