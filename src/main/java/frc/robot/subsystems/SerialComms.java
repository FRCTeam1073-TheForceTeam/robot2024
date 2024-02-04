// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Thread;  // might use sleeps at some point

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SerialComms extends SubsystemBase{

  private static SerialPort serialPort;

  public SerialComms(SerialPort.Port portUSB) {
  //   try {
  //     SerialPort serialPort = new SerialPort(1000000,port,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne)
  //     serialPort.setFlowControl(SerialPort.FlowControl.kNone);
  //   }
  //   catch (Exception e) {
  //     System.out.println("Could not open serial port!");
  //     port = null;
  //   }
  // }
    SerialPort serialPort = new SerialPort(1000000,portUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
    serialPort.setFlowControl(SerialPort.FlowControl.kNone);
  }

  public class Transmit {  
    public static String msgA = "1,a\n";
    public static String msgG = "1,g\n";
    public static String msgL = "1,l\n";
  }

  public class Recieve{
    static String msg = serialPort.readString();
  }

// this one reads 1,a from openmv
// keep around for a bit as an example of basic serial usage
  // @Override
  // public void periodic() {
  //   int recvd = 0;
  //   recvd = port.getBytesReceived();
  //   if( recvd != 0) {
  //     String incoming = port.readString();
  //     System.out.println(String.format("incoming: %s", incoming));
  //   } 
  // }
  public void parseMessage(String s){
      System.out.println(String.format("OpenMV Parse Message %s", s));
      String[] fields = s.split("[,]");
      if (fields.length % 5 != 0){  // clever use of modulo, I like it
        System.out.println("Invalid OpenMV Message");
      }else {
        System.out.println("Valid OpenMV Message");
      } 
    }

  @Override
  public void periodic() {

    //TODO: unclear how/if SerialComms will use periodic()

    // int byteswritten = serialPort.writeString(Transmit.msgA);
    // System.out.println(byteswritten);
    // System.out.println(serialPort.readString());
    // String msgR ="";  
    // Recieve.msg = msgR;  
    // parseMessage(msgR);
  }
}