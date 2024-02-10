// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends DiagnosticsSubsystem
{

  // Declares our controller variable
  public static Joystick driverController;

  public Debouncer fieldCentricDebouncer = new Debouncer(.05);
  public Debouncer parkingBrakeDebouncer = new Debouncer(0.05);
  public Debouncer menuDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer aDriverButtonDebouncer = new Debouncer(0.05);

  // Declares the "zero" value variables (which allow us to compensate for joysticks that are a little off)
  private double LEFT_X_ZERO;
  private double LEFT_Y_ZERO;
  private double RIGHT_X_ZERO;
  private double RIGHT_Y_ZERO;

  /** Creates a new OI. */
  public OI() 
  {
    // Sets the driver controller to a new joystick object at port 0
    driverController = new Joystick(0);
    zeroDriverController();
  }

  @Override
  public boolean updateDiagnostics() { 
    // TODO: Add proper diagnostics.
    return false;
  }


  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("OI");
    builder.addDoubleProperty("Driver Right Y", this::getDriverRightY, null);
    builder.addDoubleProperty("Driver Right X", this::getDriverRightX, null);
    builder.addDoubleProperty("Driver Left Y", this::getDriverLeftY, null);
    builder.addDoubleProperty("Driver Left X", this::getDriverLeftX, null);
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() 
  {

    // You can add more smartdashboard printouts here for additional joysticks or buttons
  }

  public void zeroDriverController() 
  {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = getDriverLeftX();
    LEFT_Y_ZERO = getDriverLeftY();
    RIGHT_X_ZERO = getDriverRightX();
    RIGHT_Y_ZERO = getDriverRightY();
  }

  /** The following methods return quality-controlled values from the driver controller */
  public double getDriverLeftX() 
  {
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(driverController.getRawAxis(0) - LEFT_X_ZERO, -1, 1);
  }

  public double getDriverLeftY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(1) - LEFT_Y_ZERO, -1, 1);
  }

  public double getDriverRightX() 
  {
    return MathUtil.clamp(driverController.getRawAxis(4) - RIGHT_X_ZERO, -1, 1);
  }

  public double getDriverRightY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(5) - RIGHT_Y_ZERO, -1, 1);
  }

  public double getDriverTranslateX()
  {
    return getDriverLeftX();
  }

  public double getDriverTranslateY()
  {
    return getDriverLeftY();
  }

  public double getDriverRotate()
  {
    return getDriverRightX();
  }

  /** Returns a specified button from the driver controller */
  public boolean getDriverRawButton(int i) 
  {
    return driverController.getRawButton(i);
  }

  public double getDriverRightTrigger()
  {
    return driverController.getRawAxis(3);
  }

  public double getDriverLeftTrigger()
  {
    return driverController.getRawAxis(2);
  }

  public boolean getFieldCentricToggle()
  {
    return fieldCentricDebouncer.calculate(driverController.getRawButton(7));
  }
  
  public boolean getLeftBumper()
  {
    return parkingBrakeDebouncer.calculate(driverController.getRawButton(5));
  }

  public boolean getMenuButton()
  {
    return menuDriverButtonDebouncer.calculate(driverController.getRawButton(8));
  }

  public boolean getAButton()
  {
    return aDriverButtonDebouncer.calculate(driverController.getRawButton(1));
  }
}