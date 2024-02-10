// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.io.UnsupportedEncodingException;
// import java.lang.Thread;  // might use sleeps at some point
// import java.lang.reflect.Array;
// import java.nio.charset.StandardCharsets;
// import java.util.ArrayList;

// import edu.wpi.first.wpilibj.SerialPort;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;



// public class SerialComms extends SubsystemBase{

//   private SerialPort.Port portUSB;
//   private static SerialPort serialPort;


//   public SerialComms(SerialPort.Port portUSB) {
//     this.portUSB = portUSB;
//   //   try {
//   //     SerialPort serialPort = new SerialPort(1000000,port,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne)
//   //     serialPort.setFlowControl(SerialPort.FlowControl.kNone);
//   //   }
//   //   catch (Exception e) {
//   //     System.out.println("Could not open serial port!");
//   //     port = null;
//   //   }
//   // }
//     try {
//       serialPort = new SerialPort(1000000, portUSB,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
//       serialPort.setFlowControl(SerialPort.FlowControl.kNone);
//       System.out.println("set serialPort and flowcontrol");
//     }
//     catch (Exception e) {
//       System.out.println("Could not open serial port!");
//       serialPort = null;
//     }
//   }

  

//   public static void send(byte[] message) {
//     serialPort.write(message, message.length);
//     System.out.println("Sending");
//   }

//   public static ArrayList<Byte> recieve() {
//     ArrayList<Byte> msg = new ArrayList();

//     while(1 == 1){
//       int recvd = serialPort.getBytesReceived();

//       if(recvd != 0){
//             byte[] data = serialPort.read(1);
//             msg.add(data[0]);

//             String dataString = new String(data, StandardCharsets.US_ASCII);

//             if(dataString == "\n") {
//               return msg;
//             }

//       }
      
//     }
//   }

//   public static ArrayList<Byte> getVisionData(byte[] message){
//     send(message);
//     System.out.println("Vision Data doing its thingy");
//     return recieve();
//   }


// // this one reads 1,a from openmv
// // keep around for a bit as an example of basic serial usage
//   // @Override
//   // public void periodic() {
//   //   int recvd = 0;
//   //   recvd = port.getBytesReceived();
//   //   if( recvd != 0) {
//   //     String incoming = port.readString();
//   //     System.out.println(String.format("incoming: %s", incoming));
//   //   } 
//   // }
//   public void parseMessage(String s){
//       System.out.println(String.format("OpenMV Parse Message %s", s));
//       String[] fields = s.split("[,]");
//       if (fields.length % 5 != 0){  // clever use of modulo, I like it
//         System.out.println("Invalid OpenMV Message");
//       }else {
//         System.out.println("Valid OpenMV Message");
//       } 
//     }

//   @Override
//   public void periodic() {

//     //TODO: unclear how/if SerialComms will use periodic()
//   }
// }