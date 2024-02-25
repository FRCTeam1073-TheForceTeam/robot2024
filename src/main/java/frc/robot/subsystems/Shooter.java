// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter extends DiagnosticsSubsystem{
  
  // Motors
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;

  // Motor pid values
  private double p = 0.3;
  private double i = 0;
  private double d = 0.003;

  // Motor fault trackers
  private final MotorFault topShooterMotorFault;
  private final MotorFault bottomShooterMotorFault;

  // Configurator errors
  StatusCode topConfigError;
  StatusCode bottomConfigError;

  // Motor rate limiters
  private final SlewRateLimiter topFlyWheelLimiter;
  private final SlewRateLimiter bottomFlyWheelLimiter;

  // Velocity variables
  private double targetTopVelocityMPS;
  private double targetBottomVelocityMPS;
  private double runShooterTargetTopVelocityInMPS;
  private double runShooterTargetBottomVelocityInMPS;
  private double commandedTopVelocityMPS;
  private double commandedBottomVelocityMPS;
  private double currentTopVelocityMPS;
  private double currentBottomVelocityMPS;

  // Beam break sensor
  private final DigitalInput shooterBeamBreak;

  // VelocityVoltage object
  private VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0);

  // CANbus for this subsystem
  private final String kCANbus = "CANivore";

  /* 1 motor rotation = 2 wheel rotations
   * Diameter of the wheel is 4"
   * Wheel circumference is 4π (πd)
   * Therefore, the velocity = 8π inches/rotation
   */
  private double shooterMetersPerRotation = 8 * Math.PI * 0.0254; // 0.0254 meters/inch

  /** Creates a new Shooter. **/
  public Shooter() {
    // topShooterMotor = new TalonFX(17, kCANbus); // Kraken 
    // bottomShooterMotor = new TalonFX(18, kCANbus); //Kraken 
    topShooterMotor = new TalonFX(17, kCANbus);
    bottomShooterMotor = new TalonFX(18, kCANbus);
    shooterBeamBreak = new DigitalInput(2);
    topShooterMotorFault = new MotorFault(topShooterMotor, 17);
    bottomShooterMotorFault = new MotorFault(bottomShooterMotor, 18);
    topFlyWheelLimiter = new SlewRateLimiter(8); //limits the rate of change to 1.5 units per seconds
    bottomFlyWheelLimiter = new SlewRateLimiter(8); //limits the rate of change to 1.5 units per seconds

    targetTopVelocityMPS = 0;
    targetBottomVelocityMPS = 0;

    configureHardware();
}
 
  @Override
  public void periodic() {
    updateDiagnostics();
    updateFeedback();
    // Calculate ratelimited commanded velocities in rotations/second based on meters/second target velocity
    commandedTopVelocityMPS = topFlyWheelLimiter.calculate(-targetTopVelocityMPS);
    commandedBottomVelocityMPS = bottomFlyWheelLimiter.calculate(-targetBottomVelocityMPS);
    
    // Run the motors at the current commanded velocity
    topShooterMotor.setControl(shooterVelocityVoltage.withVelocity(commandedTopVelocityMPS / shooterMetersPerRotation));
    bottomShooterMotor.setControl(shooterVelocityVoltage.withVelocity(commandedBottomVelocityMPS / shooterMetersPerRotation));
  }

  /* Updates the current motor velocities */
  public void updateFeedback(){
    currentTopVelocityMPS = topShooterMotor.getVelocity().getValue() * shooterMetersPerRotation;
    currentBottomVelocityMPS = bottomShooterMotor.getVelocity().getValue() * shooterMetersPerRotation;
  }

  /* Sets the desired motor velocites in meters per second */
  public void setTargetTopVelocityInMPS(double velocityMPS){
    targetTopVelocityMPS = velocityMPS;
  }
  public void setTargetBottomVelocityInMPS(double velocityMPS)
  {
    targetBottomVelocityMPS = velocityMPS;
  }

  /* Gets the target velocities for the motors in meters per second */
  public double getTargetTopVelocityInMPS(){
    return Math.abs(targetTopVelocityMPS);
  }
  public double getTargetBottomVelocityInMPS(){ 
    return Math.abs(targetBottomVelocityMPS);
  }

  /* Gets the ratelimited commanded velocities for the motors in rotations per second */
  public double getCommandedTopVelocityInMPS(){ 
    return Math.abs(commandedTopVelocityMPS);
  }
  public double getCommandedBottomVelocityInMPS(){ 
    return Math.abs(commandedBottomVelocityMPS);
  }

  /* Gets the actual reported velocities of the motors in rotations per second */
  public double getCurrentTopVelocityInMPS(){
    return Math.abs(currentTopVelocityMPS);
  }
  public double getCurrentBottomVelocityInMPS(){
    return Math.abs(currentBottomVelocityMPS);
  }

  public double getRunShooterTargetTopVelocityInMPS(){
    return Math.abs(runShooterTargetTopVelocityInMPS);
  }

  public double getRunShooterTargetBottomVelocityInMPS(){
    return Math.abs(runShooterTargetBottomVelocityInMPS);
  }

  public void setRunShooterTargetTopVelocityInMPS(double velocityMPS){
    runShooterTargetTopVelocityInMPS = velocityMPS;
  }

  public void setRunShooterTargetBottomVelocityInMPS(double velocityMPS){
    runShooterTargetBottomVelocityInMPS = velocityMPS;
  }

  /* Uses the beam break sensor to detect if the note has entered the shooter */
  public boolean noteIsInShooter(){
    return shooterBeamBreak.get();
  }

  public void configureHardware(){
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = p;
    configs.Slot0.kI = i;
    configs.Slot0.kD = d;
    configs.Slot0.kV = 0.12;
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    topConfigError = topShooterMotor.getConfigurator().apply(configs);
    bottomConfigError = bottomShooterMotor.getConfigurator().apply(configs);
  }

  @Override
  public boolean updateDiagnostics(){
    String result = "";
    boolean ok = false;
    if (topShooterMotorFault.hasFaults()||
    bottomShooterMotorFault.hasFaults());{
      ok = false;
      result += topShooterMotorFault.getFaults() +  bottomShooterMotorFault.getFaults();
    }
    if (!topConfigError.isOK()) 
    {
      System.err.println(String.format("TOP SHOOTER MOTOR ERROR: %s", topConfigError.toString()));
      result +=topConfigError.getDescription();
      ok = false;
    }
    if (!bottomConfigError.isOK()) 
    {
      System.err.println(String.format("BOTTOM SHOOTER MOTOR ERROR: %s", bottomConfigError.toString()));
      result +=bottomConfigError.getDescription();
      ok = false;
    }
    return setDiagnosticsFeedback(result, ok);
  }

  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("Target Top Motor Velocity", this::getTargetTopVelocityInMPS, this::setTargetTopVelocityInMPS);
    builder.addDoubleProperty("Target Bottom Motor Velocity", this::getTargetBottomVelocityInMPS, this::setTargetBottomVelocityInMPS);
    
    builder.addDoubleProperty("RunShooter Target Top Motor Velocity", this::getRunShooterTargetTopVelocityInMPS, this::setRunShooterTargetTopVelocityInMPS);
    builder.addDoubleProperty("RunShooter Bottom Motor Velocity", this::getRunShooterTargetBottomVelocityInMPS, this::setRunShooterTargetBottomVelocityInMPS);
    
    builder.addDoubleProperty("Commanded Top Motor Velocity", this::getCommandedTopVelocityInMPS, null);
    builder.addDoubleProperty("Commanded Bottom Motor Velocity", this::getCommandedBottomVelocityInMPS, null);
    builder.addDoubleProperty("Actual Top Motor Velocity", this::getCurrentTopVelocityInMPS, null);
    builder.addDoubleProperty("Actual Bottom Motor Velocity", this::getCurrentBottomVelocityInMPS, null);
  }
}