// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteFinder extends SubsystemBase {
  public class NoteData {
    //public String color = "Orange";
    public double noteYaw = 0;
    public double notePitch = 0;
    public double area = 0;
    public double timestamp = 0;

    public boolean validNote() {
      return (area >= 500); //TODO: will had a todo that was actually a good idea figure that bit out later
    }
  }

  private boolean aligned;
  public PhotonCamera note_camera = new PhotonCamera("USB_Camera");
  NoteData noteData = new NoteData();
  PhotonPipelineResult answer;

  public NoteData getCurrentNoteData(){
    return noteData;
  }

  public boolean noteFound(){
    return this.noteData.validNote();
  }

  public boolean alignment(){
    return aligned;
  }

  public NoteData readNoteData(){
    this.answer = note_camera.getLatestResult();
    if (this.answer.hasTargets()) {
      PhotonTrackedTarget bestNote = this.answer.getBestTarget();
      this.noteData.notePitch = bestNote.getPitch();
      this.noteData.noteYaw = bestNote.getYaw();
      this.noteData.timestamp = Timer.getFPGATimestamp();
    }
    return this.noteData;
  }

  public NoteFinder() {}

  @Override
  public void periodic() {
    readNoteData();

    if (Math.abs(noteData.noteYaw) <= 3.0 && noteFound()){
      aligned = true;
    }
    else {
      aligned = false;
    }
    //SmartDashboard.putNumber("AprilTag/X(Added)", this.tagData.x);
    SmartDashboard.putNumber("AprilTag/Pitch", noteData.notePitch);
    SmartDashboard.putNumber("AprilTag/Yaw", this.noteData.noteYaw);
    SmartDashboard.putBoolean("AprilTag/Valid", this.noteData.validNote()); // Allows dashboard indicator.
    SmartDashboard.putBoolean("AprilTag/Aligned-Note", aligned); 
  }
  
}