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

  SerialPort serialPort;
  byte bytes[];
  public int sendCounter = 0;
  public int recvCounter = 0;

  public SerialComms() {

    try {
      serialPort = new SerialPort(1000000, SerialPort.Port.kUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      SmartDashboard.putBoolean("SerialComms/Status", true); // Dashboard indicator.
      SmartDashboard.putString("SerialComms/Info", "Port Open"); // Dashboard indicator.

    } catch (Exception e) {
      serialPort = null;
      SmartDashboard.putBoolean("SerialComms/Status", false); // Dashboard indicator.
      SmartDashboard.putString("SerialComms/Info", "Failed To Open"); // Dashboard indicator.
    }

  }

  /**
   * Send an entire binary message to serial port.
   * @param msg
   */
  public void send(byte[] msg) {
    if (serialPort != null) {
      // SmartDashboard.putRaw("SerialComms/SendRaw", msg);
      try {
        serialPort.write(msg, msg.length);
        serialPort.flush();
        sendCounter += 1; // We sent a message.
        SmartDashboard.putNumber("SerialComms/Send", sendCounter);
      }
      catch (Exception e) {
        SmartDashboard.putBoolean("SerialComms/Status", false); // Dashboard indicator.
        SmartDashboard.putString("SerialComms/Info", "Write Exception"); // Dashboard indicator.

        try {
          serialPort.close();
        } catch (Exception ee) {
          // Nothing to do we tried.
          serialPort = null; // Drop the port object.
        }
      }
    } else {
      // TODO: Attempt to recover?
    }
  }

  /**
   * Receive all the bytes available at the serial port if >= 8 bytes and return.
   * Else return null.
   * @return
   */
  public byte[] receive() {
    if (serialPort != null) {
      try {
        int bytestoread = serialPort.getBytesReceived();
        if (bytestoread >= 8 ) {
          bytes = serialPort.read(bytestoread);
          // We got a message.
          recvCounter += 1;
          SmartDashboard.putNumber("SerialComms/Recv", recvCounter);
          return bytes;
        } else {
          return null;
        }
      } catch (Exception e) {
        SmartDashboard.putBoolean("SerialComms/Status", false);
        SmartDashboard.putString("SerialComms/Info", "Read Exception"); // Dashboard indicator.

        try {
          serialPort.close();
        } catch (Exception ee) {
          // Nothing to do... we tried.
        }
        serialPort = null; // Drop the object.
        return null;
      }
    } else {
      return null;
    }
  }
  
}

