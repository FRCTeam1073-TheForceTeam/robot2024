// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* TODO:
 * Might be nice: set to global variable instead of making a new one each time
 */

package frc.robot.subsystems;
// need to deal with angle adjustments, imputting velocity or power //

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter extends DiagnosticsSubsystem{
  
  private final TalonFX topShooterMotor;
  private final TalonFX bottomShooterMotor;
  private final MotorFault topShooterMotorFault;
  private final MotorFault bottomShooterMotorFault;
  private final DigitalInput shooterBeamBreak;
  private double offset;
  private double targetTopShooterMotorVelocityRPS;
  private double topShooterMotorVelocity;
  private double targetBottomShooterMotorVelocityRPS;
  private double bottomShooterMotorVelocity;
  private final SlewRateLimiter topFlyWheelFilter;
  private final SlewRateLimiter bottomFlyWheelFilter;
    /* 1 motor rotation = 2 wheel rotations
   * Diameter of the wheel is 4"
   * Wheel circumference is 4π (πd)
   * Therefore, the velocity = 8π inches/rotation
   */
  private double shooterMetersPerRotation = 8 * Math.PI * 0.0254; // 0.0254 meters/inch

  private double p = 0.11;
  private double i = 0.5;
  private double d = 0.0001;

  /** Creates a new Shooter. **/
  public Shooter() {
    //one motor might be a follower
    topShooterMotor = new TalonFX(17, "CANivore"); // Kracken 
    bottomShooterMotor = new TalonFX(18, "CANivore"); //Kracken 
    shooterBeamBreak = new DigitalInput(2);
    topShooterMotorFault = new MotorFault(topShooterMotor, 17);
    bottomShooterMotorFault = new MotorFault(bottomShooterMotor, 18);
    topFlyWheelFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds
    bottomFlyWheelFilter = new SlewRateLimiter(0.5); //limits the rate of change to 0.5 units per seconds


    targetTopShooterMotorVelocityRPS = 0;
    targetBottomShooterMotorVelocityRPS = 0;
    offset = 0;

    setConfigsShooter();
}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("TOF Running");
    topShooterMotorVelocity = topFlyWheelFilter.calculate(-targetTopShooterMotorVelocityRPS);
    topShooterMotor.setControl(new VelocityVoltage(topShooterMotorVelocity));
    bottomShooterMotorVelocity = bottomFlyWheelFilter.calculate(targetBottomShooterMotorVelocityRPS);
    bottomShooterMotor.setControl(new VelocityVoltage(bottomShooterMotorVelocity));
  }

/* returns true if the beam has been broken, false if there's no note in the sensor */

public void setConfigsShooter(){
  TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.Slot0.kP = p;
  configs.Slot0.kI = i;
  configs.Slot0.kD = d;
  configs.Slot0.kV = 0.12;
  configs.Voltage.PeakForwardVoltage = 8;
  configs.Voltage.PeakReverseVoltage = -8;

  configs.Slot1.kP = 5;
  configs.Slot1.kI = 0.1;
  configs.Slot1.kD = 0.001;

  configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

  topShooterMotor.getConfigurator().apply(configs);
  bottomShooterMotor.getConfigurator().apply(configs);
}



@Override
public void initSendable(SendableBuilder builder)
{
  builder.setSmartDashboardType("Shooter");
  builder.addDoubleProperty("Top Motor Velocity", this::getTopShooterMotorVelocity, this::setTopShooterMotorVelocity);
  builder.addDoubleProperty("Bottom Motor Velocity", this::getBottomShooterMotorVelocity, this::setBottomShooterMotorVelocity);
  builder.setSmartDashboardType("Tof");
}

  /* sets the desired top shooter motor velocity in rotations per second */
  public void setTopShooterMotorVelocity(double velocityMPS)
  {
    targetTopShooterMotorVelocityRPS = velocityMPS / shooterMetersPerRotation;
  }
  
  /* sets the desired bottom shooter motor velocity in rotations per second */
  public void setBottomShooterMotorVelocity(double velocityMPS)
  {
    targetBottomShooterMotorVelocityRPS = velocityMPS / shooterMetersPerRotation;
  }


  /* uses the beam break sensor to detect if the note has entered the shooter */
  public boolean noteIsInShooter(){
    return shooterBeamBreak.get();
  }

/* gets the value of the velocity that the top shooter motor is at */
public double getTopShooterMotorVelocity(){
  return targetTopShooterMotorVelocityRPS;
}

/* gets the value of the velocity that the bottom shooter motor is at */
public double getBottomShooterMotorVelocity(){ 
  return targetBottomShooterMotorVelocityRPS;
}

@Override
public boolean updateDiagnostics(){
  String result = "";
  boolean ok = true;
  if (topShooterMotorFault.hasFaults()||
  bottomShooterMotorFault.hasFaults());{
    ok = false;
  }
  result = topShooterMotorFault.getFaults() +  bottomShooterMotorFault.getFaults();
  return setDiagnosticsFeedback(result, ok);
}

}