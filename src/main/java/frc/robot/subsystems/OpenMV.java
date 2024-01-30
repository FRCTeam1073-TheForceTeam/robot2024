// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class OpenMV extends SubsystemBase{
  /** Creates a new OpenMV. */

  private SerialPort port;
  private double lastUpdateTime = 0;
  private String talk = "q";

  public OpenMV(SerialPort.Port p) {
    try {
      port = new SerialPort(2000000,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      // port.setFlowControl(SerialPort.FlowControl.kNone);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }

// this one reads 1,a from openmv
  // @Override
  // public void periodic() {
  //   int recvd = 0;
  //   recvd = port.getBytesReceived();
  //   if( recvd != 0) {
  //     String incoming = port.readString();
  //     System.out.println(String.format("incoming: %s", incoming));
  //   } 
  // }

  @Override
  public void periodic() {
    Integer wrote = port.writeString("2,g,billybuiltaguillitinetrieditonhisistrjeansaidmotherwhensh\n");
    System.out.println(String.format("just wrote this many bytes: %d", wrote));
    try {
      Thread.sleep(1000);
    }
    catch (final InterruptedException e) {
      throw new RuntimeException(e);
    }
    
  }






      // read(1);
      // System.out.println(String.format("recvd: %s", recvd));
      // byte[] thebytes = port.read(1);
      // System.out.println(String.format("byte: %d", thebytes[0]));
      // int val = thebytes[0] & 0x00ff;
      // System.out.println(val);

    //byte[] newmsg = port.read(1);
    //Byte thebyte = newmsg[0];
    // if (port != null) {
       //System.out.println("sending q");
    //   Integer wrote = port.writeString("o");
    //   System.out.println(String.format("writing buddy"));
    //if (port != null) {
      //System.out.println(thestr);
      //System.out.println();
    //int msgasint = Byte.toUnsignedInt(thebyte);
    //System.out.println(msgasint);
    //System.out.println(msg)
  // public boolean parseMessage(String s){
  //   // System.out.println(String.format("OpenMV Parse Message %s", s));
    
  //   lastUpdateTime = Timer.getFPGATimestamp(); // Update last valid time since we have a packet.
  //   System.out.println(lastUpdateTime);
  //   return true;
  // }
}