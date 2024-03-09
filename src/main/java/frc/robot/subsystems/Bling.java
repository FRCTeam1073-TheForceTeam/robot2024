// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;

public class Bling extends DiagnosticsSubsystem {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public int length = 48;
  public int slotLength;
  public int numSlots = 48;
  
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
  public Bling() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    slotLength = (int) (m_ledBuffer.getLength() / (numSlots));
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
    m_led.setData(m_ledBuffer);
    //setBatteryBling(0);
    setRGBAll(25, 5, 10);
  }

  /**
   * @return LED buffer
   */
  public AddressableLEDBuffer getM_LEDBuffer() {
    return m_ledBuffer;
  }

  /**
   * Sets one LED to a color
   * @param i - the index to write
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setRGB(int i, int r, int g, int b)
  {
    m_ledBuffer.setRGB(i, r, g, b);
  }

  /**
   * Sets all the LEDs to one color
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   * @return buffer length
   */
  public void setRGBAll(int r, int g, int b) {
    for (var i = 0; i < (m_ledBuffer.getLength()); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }

  /**
   * Turns off all LEDs
   */
  public void clearLEDs() {
    setRGBAll(0, 0, 0);
  }

   /**
    * Sets a range of LEDs to one color.
    * @param min
    * @param number
    * @param r - the r value [0-255]
    * @param g - the g value [0-255]
    * @param b - the b value [0-255]
    */
   public void setRangeRGB(int min, int number, int r, int g, int b) {
    if (number != 1) {
      int max = min + number;
      for (int i = min; i < (max); i++) {
        m_ledBuffer.setRGB(i, r, g, b);
      }
    } else {
      m_ledBuffer.setRGB(min, r, g, b);
    }
  }

  /**
   * Sets a slot of LEDs to one color.
   * @param slotNumber
   * @param r - the r value [0-255]
   * @param g - the g value [0-255]
   * @param b - the b value [0-255]
   */
  public void setSlotRGB(int slotNumber, int r, int g, int b) {
    setRangeRGB(slotNumber*slotLength, slotLength, r, g, b);

  }

  /**
   * Sets the battery bling to:
   * Green when battery voltage is greater than 12
   * Blue when battery voltage is greater than 10
   * Red when battery voltage is less than or equal to 10
   * @param slotNumber
   */
  public void setBatteryBling(int slotNumber) {
    double volts = RobotController.getBatteryVoltage();

    if (volts > 12) {
      setSlotRGB(slotNumber, 0, 255, 0);
    }
    else if (volts > 10){
      setSlotRGB(slotNumber, 0, 0, 255);
    }
    else{
      setSlotRGB(slotNumber, 255, 0, 0);
    }
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
