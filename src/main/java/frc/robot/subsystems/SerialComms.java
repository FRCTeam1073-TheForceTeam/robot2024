// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.UnsupportedEncodingException;
import java.lang.Thread;  // might use sleeps at some point
import java.lang.reflect.Array;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SerialComms extends SubsystemBase{

  private SerialPort.Port portUSB;
  private static SerialPort serialPort;


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
    //String dataString = new String(message, StandardCharsets.US_ASCII);
    //System.out.println(dataString);
    System.out.println(String.format("in send() message we're about to send as string: %s", message));
    byte[] messageAsBytes = message.getBytes(StandardCharsets.US_ASCII);
    System.out.println(String.format("message we're about to send as bytes: %s", messageAsBytes));
    System.out.println(String.format("message we're about to send as bytes converted back to ASCII string: %s", new String(messageAsBytes, StandardCharsets.US_ASCII)));
    for(int i=0; i < messageAsBytes.length; i++) {
      byte[] arrayofone = {messageAsBytes[i]};
      System.out.println(String.format("arrayofone: %s", arrayofone[0]));
      System.out.println(String.format("i: %s", i));
      serialPort.write(arrayofone, 1);
    }
    //serialPort.write(messageAsBytes, messageAsBytes.length);
    System.out.println("bottom of SerialComms.send()");
  }

  public static String receive() {
    System.out.println("in receive");
    ArrayList<Byte> msg = new ArrayList<Byte>();

    while(true) {
      int recvd = serialPort.getBytesReceived();

      if(recvd != 0){
            System.out.println("recvd was not zero");
            byte[] data = serialPort.read(1);
            System.out.println(String.format("byte data recvd: %s", data));
            msg.add(data[0]);

            // dataString is the byte we just received cast to a String so we can compare to newline without hardcoding the ord() or something
            String dataString = new String(data, StandardCharsets.US_ASCII);
            System.out.println(String.format("byte we just received as string: %s", dataString));
            if(dataString == "\n") {
              String msgAsString = msg.toString();
              System.out.println("full msg we received as string:");
              System.out.println(msgAsString);
              return msgAsString;
            }
      }
    }
  }

  public static String getVisionData(String message){
    //String messageAsASCII = new String(message, StandardCharsets.US_ASCII);
    System.out.println(String.format("sending message, as a String: %s", message));
    send(message);
    // System.out.println("Vision Data doing its thingy");
    // byte[] bytesReceived = recieve();
    //String bytesReceivedAsString = bytesReceived.toString();
    String receivedString = receive();
    System.out.println(String.format("msg received as string: %s", receivedString));
    return receivedString;
  }

  public static void sendMessage(String message){
    System.out.println(String.format("sending message, as a String: %s", message));
    send(message);
  }


  @Override
  public void periodic() {

    //TODO: unclear how/if SerialComms will use periodic()
  }
}