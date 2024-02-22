package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;

/* The diagnostics base for non-subsystem classes */
public class DiagnosticsBase implements Diagnostics {
  private boolean diagnosticsOk = true;
  private String diagnosticsDetails = "";

  /** Creates a new Diagnostics. */
  public DiagnosticsBase() {
  
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

  public boolean setDiagnosticsFeedback(String diagnosticDetails, boolean ok){
    this.diagnosticsDetails += diagnosticDetails;
    if(this.diagnosticsOk){
      this.diagnosticsOk = ok;
    }
    return this.diagnosticsOk;
  }

  public boolean printDiagnostics(boolean disabled){
    if(!this.diagnosticsOk && disabled){
      DriverStation.reportError(this.diagnosticsDetails, false);
    }
    return this.diagnosticsOk;
  }
}
