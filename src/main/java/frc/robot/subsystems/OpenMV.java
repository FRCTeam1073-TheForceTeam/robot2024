// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.util.ArrayList;
import java.lang.Thread;
import java.util.ArrayList;
import java.util.List;

//import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
// import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class OpenMV extends SubsystemBase{
  /** Creates a new OpenMV. */
  private SerialPort port;

  public OpenMV(SerialPort.Port p) {
    try {
      port = new SerialPort(2000000,p,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch (Exception e) {
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }
  public ArrayList<Byte> msg = new ArrayList();


  public void sendAprilTag() {
    byte[] cmdBytes = "1,a\n".getBytes();
    int bytesWritten = port.write(cmdBytes, cmdBytes.length);
  }

  public ArrayList<Byte> getMsg()
  {
    ArrayList<Byte> msg = new ArrayList();
    byte[] oneByte = port.read(1);
    if (oneByte == "\n".getBytes()) {
      return msg;
    }
    else {
      byte oneActualByte = oneByte[0];
      msg.add(oneActualByte);
    }
  }


  @Override
  public void periodic() {
    int bytesWaiting = port.getBytesReceived();  // returns the number of bytes waiting to be read, without actually reading them
    // byte[] cmdBytes = "2,g,0,1,2,3,4,5,6,7,8,9\n".getBytes("ASCII");
    //int cmdBytesLen = cmdBytes.length;
    //Integer wrote = port.write(cmdBytes, cmdBytesLen); // second arg is maximum bytes to write, which isn't a big deal for us
    //System.out.println(String.format("just wrote this many bytes: %d", wrote));

    try {
      Thread.sleep(1000);
    }
    catch (final InterruptedException e) {
      throw new RuntimeException(e);
    }
    //ArrayList<Byte> msg = getMsg();
    this.msg = getMsg();
    //String msgString = msg.toString();
    // char cmdChar = (char)msg[2];
    // if (cmdChar == 'a') {
    //   //do whatever apriltags do
  }
}
}
// class AprilTagPublisher {
//   // the publisher is an instance variable so its lifetime matches that of the class
//   IntegerArrayPublisher intArrayPub;

//   public void PublishAprilTag(IntegerArrayTopic intArrayTopic) {
//     // start publishing; the return value must be retained (in this case, via
//     // an instance variable)
//     //int[] someintarray = [];
//     intArrayPub = intArrayTopic.publish();

//     // publish options may be specified using PubSubOption
//     // intArrayPub = intArrayTopic.publish(PubSubOption.keepDuplicates(true));
//   }


//   public void close() {
//     // stop publishing
//     intArrayPub.close();
//   }
// }