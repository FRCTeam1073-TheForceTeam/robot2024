package frc.robot.subsystems;
import java.util.Arrays;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;


public class RangeFinder extends Diagnostics {
    SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
    byte triggerCommand[] = new byte[4];
    double range = 0.0;
    double intensity = 0.0;
    double timestamp = 0.0;

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

    public void parseData (byte message[]) {

    if (message.length < 9) {
    System.err.println("Rangefinder message format ERROR");
    return;
    }

    if (message[0] != 0x59 || message[1] != 0x59) {
     System.err.println("Rangefinder message format ERROR");
    return;
    }

    int dl = (message[2] & 0xFF);
    int dh = (message[3] & 0xFF);
    range = (dl + 255 * dh) * 0.01;
    
    int il = (message[4] & 0xFF);
    int ih = (message[5] & 0xFF);

    intensity = (il + 255 * ih) * 1.5259e-5;
    timestamp = Timer.getFPGATimestamp();
    }

    public double getRange() {
        return range;
    }

    public double getIntensity() {
        return intensity;
    }

    public double getTimestamp() {
        return timestamp;
    }

        @Override
        public void periodic() {
            int bytestoread = serialPort.getBytesReceived();
            byte bytes[];

            if (bytestoread >= 9 ) {
                bytes = serialPort.read(bytestoread);

                parseData(bytes);

                //System.out.println(Arrays.toString(bytes));
                serialPort.write(triggerCommand, 4);
            }
        }

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Rangefinder");
                builder.addDoubleProperty("Range", this::getRange, null);
                builder.addDoubleProperty("Intensity", this::getIntensity, null);
                builder.addDoubleProperty("Timestamp", this::getTimestamp, null);
            }

            @Override
            public void runDiagnostics() {
                double now = Timer.getFPGATimestamp();

            if (now - timestamp > 2.0) {
                setOK(false);
            }

            else {
                setOK(false);
            }
            }



    
        }
