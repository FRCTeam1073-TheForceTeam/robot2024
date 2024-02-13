// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector extends DiagnosticsSubsystem {

  private OI m_OI;
  private final String kCANbus = "CANivore";
  private TalonFX collectMotor = new TalonFX(14, kCANbus); // same thing
  private MotorFault collectMotorFault = new MotorFault(collectMotor, 14);
  private TalonFXConfiguration collectMotorConfigurator = new TalonFXConfiguration();
  private double targetCollectorVelocity = 1; //TODO: find appropriate speed for collector
  public VelocityVoltage collectorVelocityVoltage;

  private DigitalInput tof1;
  private DutyCycle tof1DutyCycleInput;
  private double tof1DutyCycle;
  private double tof1Freq;
  private double tof1Range;
  private final double tof1ScaleFactor = 100000; 

  private final double collectorTicksPerMeter = 1;
  private final double collectorGearRatio = 12.0/18.0;
  // private final double intakeTicksPerRadian = 2048.0 * collectorGearRatio / (2.0 * Math.PI);
  
  // fill in PID values
  
  private double collect_kP = 3;
  private double collect_kI = 0;
  private double collect_kD = 0;
  private double collect_kF = 0;

  /** Creates a new Collector. */
  public Collector(OI oi) {
    m_OI = oi;

    tof1 = new DigitalInput(0); 
    tof1DutyCycleInput = new DutyCycle(tof1);
    tof1Freq = 0;
    tof1Range = 0;

    collectorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putBoolean("Operator button 3", m_OI.getOperatorRawButton(3));
    SmartDashboard.putNumber("Converted number", (targetCollectorVelocity / (collectorGearRatio * (2 * Math.PI * 0.0254))));
    if(m_OI.getOperatorRawButton(3))
    {
      runCollectMotor(targetCollectorVelocity);
    }
    tof1Freq = tof1DutyCycleInput.getFrequency();
    tof1DutyCycle = tof1DutyCycleInput.getOutput();
    tof1Range = tof1ScaleFactor * (tof1DutyCycle / tof1Freq - 0.001);
  }
  
  public void runCollectMotor(double targetCollectorVelocity)
  {
    System.out.println("RUNNING");
    collectMotor.setControl(collectorVelocityVoltage.withVelocity(2));
  }

  public double getTargetCollectorVelocity()
  {
    return targetCollectorVelocity;
  }

  public double getActualCollectorVelocity()
  {
    return -collectMotor.getVelocity().getValueAsDouble() * collectorGearRatio * (2 * Math.PI * 0.0254); //meters per second conversion
  }

  public double getActualCollectorPosition()
  {
    return -collectMotor.getPosition().getValueAsDouble();
  }

  public double getRange1() {
    return tof1Range;
  }

  public void setRange1(double tof1Range) {
    this.tof1Range = tof1Range;
  }

  private void configureHardware(){
    //PID loop setting for collect motor
    var collectMotorClosedLoopConfig = new Slot0Configs();

    collectMotorClosedLoopConfig.withKP(collect_kP);
    collectMotorClosedLoopConfig.withKI(collect_kI);
    collectMotorClosedLoopConfig.withKD(collect_kD);
    collectMotorClosedLoopConfig.withKV(collect_kF);

    var error = collectMotor.getConfigurator().apply(collectMotorClosedLoopConfig, 0.5);
    if(!error.isOK()){
      System.err.print(String.format("Module %d COLLECT MOTOR ERROR: %s", error.toString()));
      setDiagnosticsFeedback(error.getDescription(), false);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Collector");
    builder.addDoubleProperty("Target Velocity", this::getTargetCollectorVelocity, null);
    builder.addDoubleProperty("Actual Velocity", this::getActualCollectorVelocity, null);
    builder.addDoubleProperty("Actual Position", this::getActualCollectorPosition, null);
    builder.addDoubleProperty("tof1Range", this::getRange1, null);
    // builder.addBooleanProperty("ok", this::isOK, null);
    // builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
    collectMotor.initSendable(builder);
  }

  @Override
  public boolean updateDiagnostics() 
  {
    String result = "";

    if(tof1DutyCycleInput.getFrequency()< 2){
      return setDiagnosticsFeedback(String.format("tof1 not working"), false);
    }

    return setDiagnosticsFeedback(result, true);
  }
}
