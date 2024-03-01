// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SerialComms extends SubsystemBase{
  private static SerialPort serialPort;
  private SerialPort.Port portUSB;

  public SerialComms(SerialPort.Port portUSB) {
    this.portUSB = portUSB;
    try {
      serialPort = new SerialPort(2000000, portUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
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
    //System.out.println(String.format("in send() message we're about to send as string: %s", message));
    byte[] messageAsBytes = message.getBytes(StandardCharsets.US_ASCII);
    //System.out.println(String.format("message we're about to send as bytes: %s", messageAsBytes));
    //System.out.println(String.format("message we're about to send as bytes converted back to ASCII string: %s", new String(messageAsBytes, StandardCharsets.US_ASCII)));
    for(int i=0; i < messageAsBytes.length; i++) {
      byte[] arrayofone = {messageAsBytes[i]};
      //System.out.println(String.format("arrayofone: %s", arrayofone[0]));
      //System.out.println(String.format("i: %s", i));
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
        //System.out.println(String.format("msgAsString thus far: %s", msgAsString));
        //System.out.println(String.format("byte we just received as string: |%s|", dataAsStr));
        if(dataAsChar == '\n') {
          System.out.println("should have just gotten a newline");
          System.out.println(String.format("full msg we received as string: %s", msgAsString));
          return msgAsString;
        }
      }
    }
  }
  
  static String transact(String message){
    System.out.println(String.format("sending message, as a String: %s", message));
    send(message);

    String receivedString = receive();
    System.out.println(String.format("msg received as string: %s", receivedString));
    return receivedString;
  }

  @Override
  public void periodic() {
  }
}

