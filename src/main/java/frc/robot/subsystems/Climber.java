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

  private final double gearRatio = 1.0; // TODO: Get from hardware.
  private final double winchDiameter = 0.02; // TODO: Get from hardware.
  private final double metersPerRotation = winchDiameter * Math.PI / gearRatio;

  private TalonFX left, right;   // Left and right motors wrt to robot +X forward. left is on +Y side. right is on -Y side.
  private VelocityVoltage leftVelocityVoltage, rightVelocityVoltage;
  private double leftTargetVelocity = 0.0;
  private double rightTargetVelocity = 0.0;

  private double leftPosition = 0.0;
  private double rightPosition = 0.0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    left = new TalonFX(22, kCANbus);
    right = new TalonFX(23, kCANbus);

    leftVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() {
    // Update state feedback:
    leftPosition = left.getPosition().refresh().getValue() * metersPerRotation;
    rightPosition =  right.getPosition().refresh().getValue() * metersPerRotation;
    leftVelocity = left.getVelocity().refresh().getValue() * metersPerRotation;
    rightVelocity = right.getVelocity().refresh().getValue() * metersPerRotation;


    // TODO: Check limits and don't drive past them:

    
    // Send down current command:
    left.setControl(leftVelocityVoltage.withVelocity((leftTargetVelocity / metersPerRotation)));
    right.setControl(rightVelocityVoltage.withVelocity((rightTargetVelocity / metersPerRotation)));

  }


  public void setVelocities(double leftVelocity, double rightVelocity) {
    leftTargetVelocity = leftVelocity;
    rightTargetVelocity = rightVelocity;
  }

  public double getLeftPosition() {
    return leftPosition;
  }

  public double getRightPosition() {
    return rightPosition;
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
        builder.addDoubleProperty("Target Left Vel", this::getTargetLeftVelocity, null);
        builder.addDoubleProperty("Target Right Vel", this::getTargetRightVelocity, null);
        builder.addDoubleProperty("Left Vel", this::getLeftVelocity, null);
        builder.addDoubleProperty("Right Vel", this::getRightVelocity, null);
        builder.addDoubleProperty("Left Pos", this::getLeftPosition, null);
        builder.addDoubleProperty("Right Pos", this::getRightPosition, null);
    }  

  private void configureHardware() {

        // PID Loop settings for left velocity control:
        var leftMotorClosedLoopConfig = new Slot0Configs();
        leftMotorClosedLoopConfig.withKP(1);
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
        rightMotorClosedLoopConfig.withKP(1);
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

        System.out.println("Climber configured.");

  }


}
