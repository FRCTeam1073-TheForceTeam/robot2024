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
  //   try {
  //     SerialPort serialPort = new SerialPort(1000000,port,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne)
  //     serialPort.setFlowControl(SerialPort.FlowControl.kNone);
  //   }
  //   catch (Exception e) {
  //     System.out.println("Could not open serial port!");
  //     port = null;
  //   }
  // }
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

  

  public static void send(byte[] message) {
    System.out.println(message);
    String dataString = new String(message, StandardCharsets.US_ASCII);
    System.out.println(dataString);
    serialPort.write(message, message.length);
    System.out.println("bottom of SerialComms.send()");
  }

  public static ArrayList<Byte> recieve() {
    System.out.println("in receive");
    ArrayList<Byte> msg = new ArrayList<Byte>();

    while(1 == 1){
      int recvd = serialPort.getBytesReceived();

      if(recvd != 0){
            System.out.println("recvd was not zero");
            byte[] data = serialPort.read(1);
            System.out.println("byte data recvd");
            System.out.println(data);
            msg.add(data[0]);

            String dataString = new String(data, StandardCharsets.US_ASCII);
            System.out.println("datastring");
            System.out.println(dataString);

            if(dataString == "\n") {
              return msg;
            }
      }
    }
  }

  public static ArrayList<Byte> getVisionData(byte[] message){
    String messageAsASCII = new String(message, StandardCharsets.US_ASCII);
    System.out.println("sending message, as ascii:");
    System.out.println(messageAsASCII);
    send(message);
    // System.out.println("Vision Data doing its thingy");
    return recieve();
  }


// this one reads 1,a from openmv
// keep around for a bit as an example of basic serial usage
  // @Override
  // public void periodic() {
  //   int recvd = 0;
  //   recvd = port.getBytesReceived();
  //   if( recvd != 0) {
  //     String incoming = port.readString();
  //     System.out.println(String.format("incoming: %s", incoming));
  //   } 
  // }
  // public void parseMessage(String s){
  //     System.out.println(String.format("OpenMV Parse Message %s", s));
  //     String[] fields = s.split("[,]");
  //     if (fields.length % 5 != 0){  // clever use of modulo, I like it
  //       System.out.println("Invalid OpenMV Message");
  //     }else {
  //       System.out.println("Valid OpenMV Message");
  //     } 
  //   }

  @Override
  public void periodic() {

    //TODO: unclear how/if SerialComms will use periodic()
  }
}