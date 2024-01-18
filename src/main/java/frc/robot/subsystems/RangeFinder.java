package frc.robot.subsystems;
import java.util.Arrays;
import edu.wpi.first.wpilibj.SerialPort;


public class RangeFinder extends Diagnostics {
    SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kUSB, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);

    public RangeFinder () {
    serialPort.setFlowControl(SerialPort.FlowControl.kNone);


    }
    
        @Override
        public void periodic() {
            int bytestoread = serialPort.getBytesReceived();
            byte bytes[];

            if (bytestoread >= 9 ) {
                bytes = serialPort.read(bytestoread);
                System.out.println(Arrays.toString(bytes));
            }


     }
}
