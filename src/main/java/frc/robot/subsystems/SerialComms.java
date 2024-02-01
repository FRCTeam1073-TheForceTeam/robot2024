// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;
import java.lang.annotation.Target;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SerialComms extends SubsystemBase{
  /** Creates a new OpenMV. */

  private static SerialPort port;
  private ArrayList<Target> targets;
  public int camID = 1;
  public String cam_id = Integer.toString(camID);
  public String cmd;
  public double lastUpdateTime = 0;
  public String talk = "q";

  public class Target{
    public String type;
    public int tagID = 0;
    public double imagex = 0.0;
    public double imagey = 0.0;
    public double rotation = 0.0;
    public double distance = 0.0;
    public double confidence = 0.0;
    public int amt = 0;
    
  }

  public SerialComms(SerialPort.Port p) {
    try {
      port = new SerialPort(1000000,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      // port.setFlowControl(SerialPort.FlowControl.kNone);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }

  public class Transmit {  
    public static String msgA = "1,a\n";
    public static String msgG = "1,g\n";
    public static String msgL = "1,l\n";
  }


  /*public void msgA(){
    port.writeString(Transmit.msgA);
    System.out.println(Transmit.msgA);
  }*/
  
  
  public void msgG(){
    if(camID == 2){
      port.writeString(Transmit.msgG);
      System.out.println(Transmit.msgG);
    }else{
      return;
    }
  }

  public void msgL(){
    if(camID == 3){
      port.writeString(Transmit.msgL);
      System.out.println(Transmit.msgL);
    }else{
      return;
    }
  }

  public class Recieve{
    static String msg = port.readString();
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
  public void parseMessage(String s){
      System.out.println(String.format("OpenMV Parse Message %s", s));
      String[] fields = s.split("[,]");
      if (fields.length% 5 != 0){
        System.out.println("Invaled OpenMV Message");
      }else {
        System.out.println("Valed OpenMV Message");
        for (int index = 0; index < fields.length; index += 5) {
          Target t = new Target();
          t.type = fields[index];
          t.tagID = Integer.parseInt(fields[index +1]);
          t.imagey = Double.parseDouble(fields[index +2]);
          t.imagex = Double.parseDouble(fields[index +3]);
          t.rotation = Double.parseDouble(fields[index +4]);
          t.distance = Double.parseDouble(fields[index +5]);
          t.confidence = Double.parseDouble(fields[index +6]);
          t.amt = Integer.parseInt(fields[index +7]);
          targets.add(t);
        }
      } 
    }

  @Override
  public void periodic() {
    int byteswritten = port.writeString(Transmit.msgA);
    System.out.println(byteswritten);
    System.out.println(port.readString());
    String msgR ="";  
    Recieve.msg = msgR;  
    parseMessage(msgR);
  }

  public ArrayList<Target> getTargets(){
    return targets;
  }
}