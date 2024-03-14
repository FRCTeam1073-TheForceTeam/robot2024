// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This is a serialport utility class for binary 8 byte message procotol.
 */
public class SerialComms {

  SerialPort serialPort = new SerialPort(1000000, SerialPort.Port.kUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
  
  byte bytes[];

  public SerialComms() {

  }

  /**
   * Send an entire binary message to serial port.
   * @param msg
   */
  public void send(byte[] msg) {
    SmartDashboard.putRaw("SerialCommsSendRaw", msg);
    serialPort.write(msg, msg.length);
    serialPort.flush();
  }

  /**
   * Receive all the bytes available at the serial port if >= 8 bytes and return.
   * Else return null.
   * @return
   */
  public byte[] receive() {
    int bytestoread = serialPort.getBytesReceived();
    if (bytestoread >= 8 ) {
      bytes = serialPort.read(bytestoread);
      return bytes;
    } else {
      return null;
    }
  }
  
}

