// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;


public class Bling extends DiagnosticsSubsystem {
  public AddressableLED m_ledEyes;
  public AddressableLEDBuffer m_ledBufferEyes;
  //public AddressableLED m_ledArms;
  //public AddressableLEDBuffer m_ledBufferArms;
  public Collector collector;
  public Feeder feeder;
  public Shooter shooter;
  public AprilTagFinder aprilTagFinder;

  public int eyesLength = 48;
  public int qlengthEyes = 6;
  //public int eyesNumSlots = 48;
  //public int qnum = 8;

  public int armsLength = 32;
  public int slengthArms = 16;

  
  /**
   * Creates a new bling.
   * @param key - the key name
   * @param defaultValue - the value to be returned if no value is found
   * @param length - the strip length
   * @param buffer - the buffer to write
   * @return Global default instance
   * @return The network table
   * @return Nework table entry
   * @return the entry's value or the given default value
   * @return the buffer length
   */
  public Bling(Collector collector, Feeder feeder, Shooter shooter, AprilTagFinder aprilTagFinder) {
    m_ledEyes = new AddressableLED(0);
    m_ledBufferEyes = new AddressableLEDBuffer(eyesLength);
    m_ledEyes.setLength(m_ledBufferEyes.getLength());
    m_ledEyes.setData(m_ledBufferEyes);
    m_ledEyes.start();

    // m_ledArms = new AddressableLED(1);
    // m_ledBufferArms = new AddressableLEDBuffer(armsLength);
    // m_ledArms.setLength(m_ledBufferArms.getLength());
    // m_ledArms.setData(m_ledBufferArms);
    // m_ledArms.start();

    this.collector = collector;
    this.feeder = feeder;
    this.shooter = shooter;
    this.aprilTagFinder = aprilTagFinder;
  }

  /**
   * Clears all of the LEDs on the robot.
   */
  public void initialize() {
    clearLEDs();

  }

  /**
   * Sets the buffer and runs battery bling.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ledEyes.setData(m_ledBufferEyes);
    //m_ledArms.setData(m_ledBufferArms);

    // if(!DriverStation.isDisabled()){
      setBatteryBling();

      setSectionRGB(0, 0, 255, 0);
      setSectionRGB(1, 0, 255, 0);

      if(aprilTagFinder.isAligned()){
        setAlignedBling();
      }
      else{
        setUnalignedBling();
      }
      // boolean shooterBling = setRainbowBling();
      double tofFeederValue = feeder.getTofRange();
      if(collector.hasNote()){
        setCollectedBling();
      }
      else if(tofFeederValue <= 0.2){
        if(shooter.getCurrentTopVelocityInMPS() > (shooter.getTargetTopVelocityInMPS() - 2)
          && shooter.getCurrentTopVelocityInMPS() > 20
          && shooter.getCurrentBottomVelocityInMPS() > (shooter.getTargetBottomVelocityInMPS() - 2)){
            setShooterBling();
        }
        else{
          setFeededBling();
        }
      }
      else{
        setNoNoteBling();
      }
    // }
    // if (DriverStation.isDisabled()){
      //setRainbowBling();
    // }
  }

  /**
   * @return LED buffer
   */
  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBufferEyes;
  }

  /**
   * Sets one LED to a color
   * @param i - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setEyesRGB(int i, int r, int g, int b)
  {
    m_ledBufferEyes.setRGB(i, r, g, b);
  }
  
  // public void setArmsRGB(int i, int r, int g, int b)
  // {
  //   m_ledBufferArms.setRGB(i, r, g, b);
  // }

  /**
   * Sets all the LEDs to one color
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   * @return buffer length
   */
  public void setEyesRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBufferEyes.getLength()); i++) {
      m_ledBufferEyes.setRGB(i, r, g, b);
    }
  }

  // public void setArmsRGBAll(int r, int g, int b) {
  //   for (var i = 0; i < (m_ledBufferArms.getLength()); i++) {
  //     m_ledBufferArms.setRGB(i, r, g, b);
  //   }
  // }

  /**
   * Turns off all LEDs
   */
  public void clearLEDs() {
    setEyesRGBAll(0, 0, 0);
    //setArmsRGBAll(0, 0, 0);
  }

   /**
    * Sets a range of LEDs to one color.
    * @param min
    * @param number
    * @param r - the r value [0-255]
    * @param g - the g value [0-255]
    * @param b - the b value [0-255]
    */
   public void setEyesRangeRGB(int min, int number, int r, int g, int b) {
    if (number != 1) {
      int max = min + number;
      for (int i = min; i < (max); i++) {
        m_ledBufferEyes.setRGB(i, r, g, b);
      }
    } else {
      m_ledBufferEyes.setRGB(min, r, g, b);
    }
  }

  // public void setArmsRangeRGB(int min, int number, int r, int g, int b) {
  //   if (number != 1) {
  //     int max = min + number;
  //     for (int i = min; i < (max); i++) {
  //       m_ledBufferArms.setRGB(i, r, g, b);
  //     }
  //   } else {
  //     m_ledBufferArms.setRGB(min, r, g, b);
  //   }
  // }

  // first quadrant is 0, second is 1, third is 2, etc...
  public void setQuadRGB(int quad, int r, int g, int b){
    setEyesRangeRGB((quad * qlengthEyes), qlengthEyes, r, g, b);
  }

  public void setSectionRGB(int sect, int r, int g, int b){
    //setArmsRangeRGB((sect * slengthArms), slengthArms, r, g, b);
  }

  /**
   * Sets the battery bling to:
   * Green when battery voltage is greater than 12
   * Blue when battery voltage is greater than 10
   * Red when battery voltage is less than or equal to 10
   * @param quadNum */
  public void setBatteryBling() {
    double volts = RobotController.getBatteryVoltage();

    if (volts > 12) {
      setQuadRGB(1, 0, 255, 0);
      setQuadRGB(2, 0, 255, 0);
    }
    else if (volts > 10){
      setQuadRGB(1, 0, 0, 255);
      setQuadRGB(2, 0, 0, 255);
    }
    else{
      setQuadRGB(1, 255, 0, 0);
      setQuadRGB(2, 255, 0, 0);
    }
  }

  /**
   * Sets the collector bling to orange.
   */
  public void setCollectedBling() {
    setQuadRGB(4, 0, 0, 0);
    setQuadRGB(5, 0, 0, 0);
    setQuadRGB(6, 85, 55, 0);
    setQuadRGB(7, 85, 55, 0);
  }

  /**
   * Sets the feeder bling to orange.
   */
  public void setFeededBling() {
    setQuadRGB(4, 85, 55, 0);
    setQuadRGB(5, 85, 55, 0);
    setQuadRGB(6, 85, 55, 0);
    setQuadRGB(7, 85, 55, 0);
  }

  /**
   * Clears the note bling ring.
   */
  public void setNoNoteBling(){
    setQuadRGB(4, 2, 0, 0);
    setQuadRGB(5, 2, 0, 0);
    setQuadRGB(6, 2, 0, 0);
    setQuadRGB(7, 2, 0, 0);
  }

  /**
   * Sets the note bling ring to green.
   */
  public void setShooterBling(){
    setQuadRGB(4, 0, 255, 0);
    setQuadRGB(5, 0, 255, 0);
    setQuadRGB(6, 0, 255, 0);
    setQuadRGB(7, 0, 255, 0);
  }

  /**
   * TODO:// get vision people to fill this out???
   * Sets the aligned bling to:
   * Green when Note is aligned?
   * Red when Note isn't aligned?
   * @param quadNum */
  public void setAlignedBling() {
    //  setQuadRGB(quadNum1, 255, 0, 0);
    //  setQuadRGB(quadNum2, 255, 0, 0);
     setQuadRGB(0, 0, 255, 0);
     setQuadRGB(3, 0, 255, 0);
  }

  public void setUnalignedBling() {
     setQuadRGB(0, 0, 0, 0);
     setQuadRGB(3, 0, 0, 0);
  }

  // public void setRainbowBling(){
  //   int m_rainbowFirstPixelHue1 = 0;
  //   int m_rainbowFirstPixelHue2 = 0;
  //   int count2 = 0;

  //   for (var i = 0; i < m_ledBufferArms.getLength()/2; i++) {
  //     final var hue1 = (m_rainbowFirstPixelHue1 + (i * 180 / m_ledBufferEyes.getLength()*2)) % 180;
  //     // Set the value
  //     m_ledBufferEyes.setHSV(i, hue1, 255, 128);

  //     // Increase by to make the rainbow "move"
  //     m_rainbowFirstPixelHue1 += 3;
  //     // Check bounds
  //     m_rainbowFirstPixelHue1 %= 180;
  //   }

  //   for (var j = m_ledBufferArms.getLength()/2; j < m_ledBufferArms.getLength(); j++) {
  //     final var hue2 = (m_rainbowFirstPixelHue2 + (count2 * 180 / m_ledBufferEyes.getLength()*2)) % 180;
  //     m_ledBufferEyes.setHSV(j, hue2, 255, 128);
  //     count2++;
  //   }
  //   m_rainbowFirstPixelHue2 += 3;
  //   m_rainbowFirstPixelHue2 %= 180;
      
  // }

  public void setArmsBling() {
     setQuadRGB(0, 0, 0, 0);
     setQuadRGB(3, 0, 0, 0);
  }

  // Initialize preferences for this class:
  public static void initPreferences() {
    //TODO: remove if not using preferences
  }
  
  /**
   * @return diagnostic results
   */
  @Override
  public boolean updateDiagnostics() {
    String result = new String();
    boolean isOK = true;
    
    //TODO: run diagnostics here
    return setDiagnosticsFeedback(result, isOK);
  }
}
