// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpenMV extends SubsystemBase {
  /** Creates a new openMV. */

  private SerialPort port;
  private String talk = "q";

  public OpenMV(SerialPort.Port p) {
    try {
      port = new SerialPort(2000000, p, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
      port.setFlowControl(SerialPort.FlowControl.kNone);
    }
    catch(Exception e){
      System.out.println("OpenMV Could not open serial port!");
      port = null;
    }
  }

  @Override
  public void periodic() {
    System.out.println("it works pls pls pls pls it works pls");
    /*   System.out.println("sending q");
      Integer wrote = port.writeString(talk);
      System.out.println(String.format("wrote: %s", wrote));
      try{
        Thread.sleep(1000);
      }
      catch(final InterruptedException e){
        throw new RuntimeException(e); */
      }

  }

