// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.HashMap;

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
  /** Creates a new Camera. */
  public Camera() {
    HashMap<String, Integer> camStats = new HashMap<String, Integer>();
    camStats.put("baudrate", 926000);
    camStats.put("timeStamp", 54);

    HashMap<String, Double> aprilTags = new HashMap<String, Double>();
    aprilTags.put("X Size", 4.0);
    aprilTags.put("Y Size", 5984.0);
    aprilTags.put("Rotation", 5924.0);
    aprilTags.put("Distance", 5984.0);
    aprilTags.put("quality", 5984.0);
  
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

  }
  
  public void SerialComms() {
    this.route = this.route;
    this.send = this.send;
    this.receive = this.receive;

    HashMap<String, Integer> send = new HashMap<String, Integer>();
    send.put ("camera.ID", route);
    


    HashMap<String, String> sendMessage = new HashMap<String, String>();
    sendMessage.put("Message", "<CameraID, cmd(AT, GP, LD, OR), (TagID, CenterX, CenterY, Rotation, Distance, Quality)* Tag Number"); 

    HashMap<String, String> recieveMessage = new HashMap<String, String>();
    recieveMessage.put("Message", "3, AT, (9, 3.33, 3.33, 0.0, 1.35, 50)* 5"); //TODO Change Numbers
  }

  @Override
  public void periodic() {
    
  }
}
