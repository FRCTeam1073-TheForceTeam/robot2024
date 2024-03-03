// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO: check slew rate limiters, initialize to the right angle, 
change methods to use pivot in radians */

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Pivot extends DiagnosticsSubsystem {

  // Motor and related objects
  private final TalonFX pivotMotor;
  private final MotorFault pivotMotorFault;
  private final SlewRateLimiter pivotMotorFilter;
  private StatusCode configError;

  // Motor scale factors
  private final double pivotGearRatio = 16 * 8.18 / 1;
  private final double pivotWheelRadius = 0.1429; //meters
  private final double pivotMeterPerRotations = pivotWheelRadius * 2 * Math.PI * pivotGearRatio;
  private double pivotRotationsPerRadian = -pivotGearRatio / (2 * Math.PI);

  // Pivot limits
  private final double minAngleRad = -.88;
  private final double maxAngleRad = 0;

  // Motor pid values
  private double p = 5;
  private double i = 2;
  private double d = 0.0;

  // Position variables in rotations
  private double testPositionRad;
  private double targetPositionRad;
  private double commandedPositionRad;
  private double currentPositionRad;

  private double debugPivotAngle;

  // PositionVoltage object
  //private PositionVoltage pivotPositionVoltage = new PositionVoltage(0).withSlot(0);
  private MotionMagicVoltage pivotPositionVoltage = new MotionMagicVoltage(0).withSlot(0);

  // CANbus for this subsystem
  private final String kCANbus = "CANivore";

  /** Creates a new Pivot. */
  public Pivot() {
    //pivotMotor = new TalonFX(21, kCANbus); //Falcon
    pivotMotor = new TalonFX(21, kCANbus);
    pivotMotorFault = new MotorFault(pivotMotor, 21);
    pivotMotorFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    pivotMotor.setPosition(0); //TODO - initialize position

    targetPositionRad = 0;

    configureHardware();
  }

  @Override
  public void periodic() {
    updateDiagnostics();
    updateFeedback();
    // This method will be called once per scheduler run
    //commandedPositionRad = pivotMotorFilter.calculate(MathUtil.clamp(targetPositionRad, minAngleRad, maxAngleRad));
    commandedPositionRad = MathUtil.clamp(targetPositionRad, minAngleRad, maxAngleRad);
    pivotMotor.setControl(pivotPositionVoltage.withPosition(commandedPositionRad * pivotRotationsPerRadian));
  }

  /* Updates the current motor position */
  public void updateFeedback(){
    currentPositionRad = pivotMotor.getPosition().getValue() / pivotRotationsPerRadian;
  }

  /* Sets the desired motor position in radians */
  public void setTargetPositionInRad(double pivotMotorPositionRad)
  {
    targetPositionRad = pivotMotorPositionRad;
  }

  public double getTestCommandTargetPositionInRad()
  {
    return testPositionRad;
  }

  /* Gets the desired motor position in radians */
  public double getTargetPositionInRad()
  {
    return targetPositionRad;
  }

  /* Gets the ratelimited commanded velocity for the motor in rotations per second */
  public double getCommandedPositionInRad(){
    return commandedPositionRad;
  }

  /* Gets the actual reported rotation of the motor in radians */
  public double getCurrentPositionInRad(){
    return currentPositionRad;
  }

  public void configureHardware(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = p;
    configs.Slot0.kI = i;
    configs.Slot0.kD = d;
    // configs.Slot0.kV = 0.12;
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    configs.MotionMagic.MotionMagicCruiseVelocity = 16;
    configs.MotionMagic.MotionMagicAcceleration = 15;
    configs.MotionMagic.MotionMagicJerk = 0;
    // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    //configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configError = pivotMotor.getConfigurator().apply(configs);
  }

  @Override
  public boolean updateDiagnostics(){
    String result = "";
    boolean ok = true;
    if (
    pivotMotorFault.hasFaults()){
      ok = false;
      result += pivotMotorFault.getFaults();
    }
    if (!configError.isOK()) 
    {
      System.err.println(String.format("PIVOT MOTOR ERROR: %s", configError.toString()));
      result += configError.getDescription();
      ok = false;
    }
    return setDiagnosticsFeedback(result, ok);
  }

  public double getDebugPivotAngle(){
    return debugPivotAngle;
  }

  public void setDebugPivotAngle(double angle){
    debugPivotAngle = angle;
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Pivot");
    builder.addDoubleProperty("Debug Pivot Angle", this::getDebugPivotAngle, this::setDebugPivotAngle);
    builder.addDoubleProperty("Pivot Test Command Motor Position", this::getTargetPositionInRad, null);
    builder.addDoubleProperty("Target Pivot Motor Position", this::getTargetPositionInRad, this::setTargetPositionInRad);
    builder.addDoubleProperty("Commanded Pivot Motor Position", this::getCommandedPositionInRad, null);
    builder.addDoubleProperty("Actual Pivot Motor Position", this::getCurrentPositionInRad, null);
  }
}
