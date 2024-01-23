// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class OpenMV extends SubsystemBase{
  /** Creates a new OpenMV. */

  private SerialPort port;
  private double lastUpdateTime = 0;
  private String talk = "q";

  public OpenMV(SerialPort.Port p) {
    try {
      port = new SerialPort(921600,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      // port.setFlowControl(SerialPort.FlowControl.kNone);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }

  @Override
  public void periodic() {
    if (port != null) {
      System.out.println("sending q");
      Integer wrote = port.writeString(talk);
      System.out.println(String.format("wrote: %s", wrote));
      // System.out.println(String.format("bytesout %s", bytesout));
      // System.out.println(String.format("talklen: %s",talk.length()));
      // System.out.println("Hellow Third World");
    }     
  }

  // public boolean parseMessage(String s){
  //   // System.out.println(String.format("OpenMV Parse Message %s", s));
    
  //   lastUpdateTime = Timer.getFPGATimestamp(); // Update last valid time since we have a packet.
  //   System.out.println(lastUpdateTime);
  //   return true;
  // }
}