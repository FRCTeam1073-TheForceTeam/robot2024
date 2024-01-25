// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.HashMap;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  int id;
  Dictionary camStats;
  Dictionary aprilTags;
  Dictionary gamePiece;
  Dictionary lineDetection;
  int route;
  Dictionary send;
  Dictionary receive;
  private SerialPort port;
  private double lastUpdateTime = 0;
  private String talk = "q";
  /** Creates a new Camera. */
  public Camera(SerialPort.Port p) {
    HashMap<String, Integer> camStats = new HashMap<String, Integer>();
    camStats.put("baudrate", 926000);
    camStats.put("timeStamp", 54);

    HashMap<String, Double> aprilTags = new HashMap<String, Double>();
    aprilTags.put("Center X", 4.0);
    aprilTags.put("Center Y", 5984.0);
    aprilTags.put("Rotation", 5924.0);
    aprilTags.put("Distance", 5984.0);
    aprilTags.put("Quality", 5984.0);
    aprilTags.put("Num of Tags", 1.0);
  
    HashMap<String, String> lineDetectionColor = new HashMap<String, String>();
    lineDetectionColor.put("Color", "Red");

    HashMap<String, Double> lineDetectionNumber = new HashMap<String, Double>();
    lineDetectionNumber.put("Distance", 1.5);
    lineDetectionNumber.put("quality", 1.0);
    lineDetectionNumber.put("Rotation", 2.0);
    
    HashMap<String, Double> gamePiece = new HashMap<String, Double>();
    gamePiece.put("X", 2.0);
    gamePiece.put("Y", 2.0);
    gamePiece.put("Distance", 2.0);
    gamePiece.put("quality", 2.0);

    this.id = this.id;
    this.camStats = this.camStats;
    this.aprilTags = this.aprilTags;
    this.lineDetection = this.lineDetection;
     this.gamePiece = this.gamePiece;


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
  
  public void SerialComms() {
    this.route = this.route;
    this.send = this.send;
    this.receive = this.receive;

    //HashMap<String, Integer> send = new HashMap<String, Integer>();
    //send.put ("camera.ID", route);
    
    //HashMap<String, String> sendMessage = new HashMap<String, String>();
    //sendMessage.put("Message", "1,A"); 

    //HashMap<String, String> recieveMessage = new HashMap<String, String>();
    //recieveMessage.put("Message", "1, A, CenterX, CenterY, Distance, Quality"); //TODO Change Numbers
  }

  

  @Override
  public void periodic() {
    if (port != null) {
      System.out.println("sending q");
      Integer wrote = port.writeString("1,A,\n");
      System.out.println(String.format("wrote: %s", wrote));
      String read = port.readString();
      System.out.println(read);
    }
  }
}
