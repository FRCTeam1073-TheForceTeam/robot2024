// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Climber extends DiagnosticsSubsystem {
  private final String kCANbus = "CANivore";
  private final OI m_OI;

  private final double gearRatio = 21; // TODO: Get from hardware.
  private final double winchDiameter = 0.01905; // TODO: Get from hardware. .75 inches
  private final double rightMetersPerRotation = winchDiameter * Math.PI / gearRatio;
  private final double leftMetersPerRotation = -winchDiameter * Math.PI / gearRatio;

  private TalonFX left, right;   // Left and right motors wrt to robot +X forward. left is on +Y side. right is on -Y side.
  private VelocityVoltage leftVelocityVoltage, rightVelocityVoltage;
  private PositionVoltage leftPositionVoltage, rightPositionVoltage;
  private double leftTargetPosition = 0.0;
  private double rightTargetPosition = 0.0;
  private double leftTargetVelocity = 0.0;
  private double rightTargetVelocity = 0.0;

  private double leftPosition = 0.0;
  private double rightPosition = 0.0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  private double minLeftPosition = 0;
  private double minRightPosition = 0;
  private double maxLeftPosition = 0.3;
  private double maxRightPosition = 0.3;

  /** Creates a new Climber. */
  public Climber(OI oi) {
    left = new TalonFX(22, kCANbus);
    right = new TalonFX(23, kCANbus);

    leftVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftPositionVoltage = new PositionVoltage(0).withSlot(0);
    rightPositionVoltage = new PositionVoltage(0).withSlot(0);

    leftTargetPosition = minLeftPosition;
    rightTargetPosition = minRightPosition;

    configureHardware();
    m_OI = oi;
  }

  @Override
  public void periodic() {
    // Update state feedback:
    leftPosition = left.getPosition().refresh().getValue() * leftMetersPerRotation;
    rightPosition =  right.getPosition().refresh().getValue() * rightMetersPerRotation;
    leftVelocity = left.getVelocity().refresh().getValue() * leftMetersPerRotation;
    rightVelocity = right.getVelocity().refresh().getValue() * rightMetersPerRotation;


    // TODO: Check limits and don't drive past them:

    
    // Send down current command:
    if(!m_OI.getCollectMode()){
      left.setControl(leftVelocityVoltage.withVelocity((leftTargetVelocity / leftMetersPerRotation)));
      //right.setControl(rightVelocityVoltage.withVelocity((rightTargetVelocity / rightMetersPerRotation)));
    }
    else{
      left.setControl(leftPositionVoltage.withPosition((leftTargetPosition / leftMetersPerRotation)));
      //right.setControl(rightPositionVoltage.withPosition((rightTargetPosition / rightMetersPerRotation)));
    }
  }

  /**
   * @param leftVelocity velocity is positive when climber is going up
   * @param rightVelocity velocity is positive when climber is going up
   */
  public void setVelocities(double leftVelocity, double rightVelocity) {
    if(leftPosition <= minLeftPosition){
      leftTargetVelocity = Math.max(leftVelocity, 0);
    }
    else if(leftPosition >= maxLeftPosition){
      leftTargetVelocity = Math.min(leftVelocity, 0);
    }
    else{
      leftTargetVelocity = leftVelocity;
    }

    if(rightPosition <= minRightPosition){
      rightTargetVelocity = Math.max(rightVelocity, 0);
    }
    else if(rightPosition >= maxRightPosition){
      rightTargetVelocity = Math.min(rightVelocity, 0);
    }
    else{
      rightTargetVelocity = rightVelocity;
    }

    // leftTargetVelocity = leftVelocity;
    // rightTargetVelocity = rightVelocity;
  }

  public void setPositions(double leftPosition, double rightPosition){
    if(leftPosition < minLeftPosition){
      leftTargetPosition = minLeftPosition;
    }
    else if(leftPosition > maxLeftPosition){
      leftTargetPosition = maxLeftPosition;
    }
    else{
      leftTargetPosition = leftPosition;
    }

    if(rightPosition < minRightPosition){
      rightTargetPosition = minRightPosition;
    }
    else if(rightPosition > maxRightPosition){
      rightTargetPosition = maxRightPosition;
    }
    else{
      rightTargetPosition = rightPosition;
    }
  }

  public double getLeftPosition() {
    return leftPosition;
  }

  public double getRightPosition() {
    return rightPosition;
  }

  public double getTargetLeftPosition() {
    return leftTargetPosition;
  }

  public double getTargetRightPosition() {
    return rightTargetPosition;
  }

  public double getLeftVelocity() {
    return leftVelocity;
  }

  public double getRightVelocity() {
    return rightVelocity;
  }

  public double getTargetLeftVelocity() {
    return leftTargetVelocity;
  }

  public double getTargetRightVelocity() {
    return rightTargetVelocity;
  }

  @Override
    public boolean updateDiagnostics() {

        MotorFault leftFault = new MotorFault(left,22);
        if (leftFault.hasFaults()) {
            return setDiagnosticsFeedback(leftFault.getFaults(), false);
        }

        MotorFault rightFault = new MotorFault(right,23);
        if (rightFault.hasFaults()) {
            return setDiagnosticsFeedback(rightFault.getFaults(), false);
        }

        return setDiagnosticsFeedback("OK", true);
    }

   @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        // builder.addDoubleProperty("Target Left Vel", this::getTargetLeftVelocity, null);
        // builder.addDoubleProperty("Target Right Vel", this::getTargetRightVelocity, null);
        // builder.addDoubleProperty("Left Vel", this::getLeftVelocity, null);
        // builder.addDoubleProperty("Right Vel", this::getRightVelocity, null);
        builder.addDoubleProperty("Left Pos", this::getLeftPosition, null);
        builder.addDoubleProperty("Right Pos", this::getRightPosition, null);
        builder.addDoubleProperty("Target Left Pos", this::getTargetLeftPosition, null);
        builder.addDoubleProperty("Target Right Pos", this::getTargetRightPosition, null);
    }  

  private void configureHardware() {

        // PID Loop settings for left velocity control:
        var leftMotorClosedLoopConfig = new Slot0Configs();
        leftMotorClosedLoopConfig.withKP(0.3);
        leftMotorClosedLoopConfig.withKI(0.0);
        leftMotorClosedLoopConfig.withKD(0.0);
        leftMotorClosedLoopConfig.withKV(0.05);

        var error = left.getConfigurator().apply(leftMotorClosedLoopConfig, 0.5);
        if (error.isOK()) {
            System.err.println(String.format("Left Climber Motor Configuration Error: %s", error.getDescription()));
            setDiagnosticsFeedback(error.getDescription(), false);
        }

        // PID Loop settings for right velocity control:
        var rightMotorClosedLoopConfig = new Slot0Configs();
        rightMotorClosedLoopConfig.withKP(0.3);
        rightMotorClosedLoopConfig.withKI(0.0);
        rightMotorClosedLoopConfig.withKD(0.0);
        rightMotorClosedLoopConfig.withKV(0.05);

        error = right.getConfigurator().apply(rightMotorClosedLoopConfig, 0.5);
        if (error.isOK()) {
            System.err.println(String.format("Right Climber Motor Configuration Error: %s", error.getDescription()));
            setDiagnosticsFeedback(error.getDescription(), false);
        }

        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);

        left.setPosition(0);
        right.setPosition(0);

        System.out.println("Climber configured.");

  }


}
