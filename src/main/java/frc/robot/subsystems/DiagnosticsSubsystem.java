// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* The diagnostics base for subsystems */
public class DiagnosticsSubsystem extends SubsystemBase implements Diagnostics {
  private boolean diagnosticsOk = true;
  private String diagnosticsDetails = "";

  /** Creates a new Diagnostics. */
  public DiagnosticsSubsystem() {
  
  }

  @Override
  public boolean updateDiagnostics() { 
    return false;
  }

  @Override 
  public boolean diagnosticsOk() {
    return this.diagnosticsOk;
  }

  @Override
  public String getDiagnosticsDetails(){
    return this.diagnosticsDetails;
  }

  public boolean setDiagnosticsFeedback(String diagnosticDetails, boolean ok) {
    this.diagnosticsDetails += diagnosticDetails;
    if(this.diagnosticsOk){
      this.diagnosticsOk = ok;
    }
    return this.diagnosticsOk;
  }

  public boolean printDiagnostics(boolean disabled){
    if(!this.diagnosticsOk && disabled){
      System.out.println(this.diagnosticsDetails);
    }
    return this.diagnosticsOk;
  }
}
