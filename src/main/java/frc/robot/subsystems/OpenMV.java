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



public class OpenMV extends SubsystemBase{
  /** Creates a new OpenMV. */
  private SerialPort port;

  public OpenMV(SerialPort.Port p) {
    try {
      port = new SerialPort(2000000,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }


  public void getAprilTag() {
    byte[] cmdBytes = "1,a\n".getBytes();
    int bytesWritten = port.write(cmdBytes, cmdBytes.length);

  }

  public ArrayList<Byte> getMsg()
  {
    ArrayList<Byte> msg = new ArrayList();
    byte[] oneByte = port.read(1);
    if (oneByte == "\n".getBytes()) {
      return msg.toArray(new byte[0]);
    }
    else {
      byte oneActualByte = oneByte[0];
      msg.add(oneActualByte);
    }
  }


  @Override
  public void periodic() {
    int bytesWaiting = port.getBytesReceived();  // returns the number of bytes waiting to be read, without actually reading them
    byte[] cmdBytes = "2,g,0,1,2,3,4,5,6,7,8,9\n".getBytes("ASCII");
    int cmdBytesLen = cmdBytes.length;
    Integer wrote = port.write(cmdBytes, cmdBytesLen); // second arg is maximum bytes to write, which isn't a big deal for us
    System.out.println(String.format("just wrote this many bytes: %d", wrote));
    try {
      Thread.sleep(1000);
    }
    catch (final InterruptedException e) {
      throw new RuntimeException(e);
    }
    ArrayList<Byte> msg = getMsg();
    //String msgString = msg.toString();
    char cmdChar = (char)msg[2];
    if (cmdChar == 'a') {
      //do whatever apriltags do
  }
}







      // read(1);
      // System.out.println(String.format("recvd: %s", recvd));
      // byte[] thebytes = port.read(1);
      // System.out.println(String.format("byte: %d", thebytes[0]));
      // int val = thebytes[0] & 0x00ff;
      // System.out.println(val);
