// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SerialComms extends SubsystemBase{
  private static SerialPort serialPort;
  private SerialPort.Port portUSB;

  public SerialComms(SerialPort.Port portUSB) {
    this.portUSB = portUSB;
    try {
      serialPort = new SerialPort(1000000, portUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      serialPort.setFlowControl(SerialPort.FlowControl.kNone);
      System.out.println("set serialPort and flowcontrol");
    }
    catch (Exception e) {
      System.out.println("Could not open serial port!");
      serialPort = null;
    }
  }

  public static void send(String message) {
    if(serialPort == null){
      return;
    }
    message = message.concat("\n");
    byte[] messageAsBytes = message.getBytes(StandardCharsets.US_ASCII);
    for(int i=0; i < messageAsBytes.length; i++) {
      byte[] arrayofone = {messageAsBytes[i]};
      serialPort.write(arrayofone, 1);
    }
  }

  public static String receive() {
    if(serialPort == null){
      return "";
    }
    System.out.println("in receive");
    //ArrayList<Byte> msg = new ArrayList<Byte>();
    String msgAsString = new String();

    while(true) {
      int recvd = serialPort.getBytesReceived();

      if(recvd != 0){
        //System.out.println("recvd was not zero");
        byte[] data = serialPort.read(1);
        String dataAsStr = new String(data, StandardCharsets.US_ASCII);
        char dataAsChar = new String(data, StandardCharsets.US_ASCII).toCharArray()[0];

        msgAsString = msgAsString.concat(dataAsStr);
        if(dataAsChar == '\n') {  
          System.out.println("should have just gotten a newline");
          System.out.println(String.format("full msg we received as string: %s", msgAsString));
          SmartDashboard.putString("SerialCommReceivedMsg", msgAsString);
          return msgAsString;
        }
      }
      else {
        return "";
      }
    }
  }
  
  @Override
  public void periodic() {
  }
}

