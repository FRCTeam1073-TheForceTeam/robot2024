package frc.robot.subsystems;
import java.util.Arrays;
import edu.wpi.first.wpilibj.SerialPort;


public class RangeFinder extends Diagnostics {
    SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kUSB, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte triggerCommand[] = new byte[4];

    public RangeFinder () {
    serialPort.setFlowControl(SerialPort.FlowControl.kNone);
    
    triggerCommand[0] = 0x5a;
    triggerCommand[1] = 0x04;
    triggerCommand[2] = 0x04;
    triggerCommand[3] = 0x62;

    Handshake();
    }

    public void Handshake () {
    byte FrameCommand[] = new byte[6];
    FrameCommand[0] = 0x5A;
    FrameCommand[1] = 0x06;
    FrameCommand[2] = 0x03;
    FrameCommand[3] = 0x0;
    FrameCommand[4] = 0x0;
    FrameCommand[5] = 0x63;
    serialPort.write(FrameCommand, 6);
    serialPort.flush();
    try {
        Thread.sleep(50);
         }
    catch (Exception e) {
        System.out.println(e);
         }
    serialPort.reset();
    serialPort.write(triggerCommand, 4);
    }


    
        @Override
        public void periodic() {
            int bytestoread = serialPort.getBytesReceived();
            byte bytes[];

            if (bytestoread >= 9 ) {
                bytes = serialPort.read(bytestoread);
                System.out.println(Arrays.toString(bytes));
                serialPort.write(triggerCommand, 4);
            }



     }
}
