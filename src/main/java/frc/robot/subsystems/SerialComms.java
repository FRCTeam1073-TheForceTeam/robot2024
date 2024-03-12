// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SerialComms extends SubsystemBase{
  SerialPort serialPort = new SerialPort(1000000, SerialPort.Port.kUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
  
  byte bytes[];

  public SerialComms() {
  }

  public void send(byte[] msg) {
    serialPort.write(msg, 8);
    serialPort.flush();
  }

  public byte[] receive(){
    int bytestoread = serialPort.getBytesReceived();
    if (bytestoread >= 8 ) {
      bytes = serialPort.read(8);
    }
    return bytes;
  }

  
  
  @Override
  public void periodic() {
  }
}

