// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StartRecordingAutonomous;
import frc.robot.commands.StartRecordingTeleop;
import frc.robot.commands.StopRecording;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.SerialComms;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OpenMV;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SerialPort.Port serial_port = SerialPort.Port.kUSB;
  // The robot's subsystems and commands are defined here...
  private final SerialComms m_serial = new SerialComms(SerialPort.Port.kUSB);
  private final Camera m_camera1 = new Camera(m_serial, 2);  // camID is how SerialComms and the cameras themselves tells them apart
  //private final GetTagData c_GetTagData = new GetTagData(m_camera1);
  private final StartRecordingAutonomous c_startRecordingAutonomous = new StartRecordingAutonomous(m_camera1);
  private final StartRecordingTeleop c_startRecordingTeleop = new StartRecordingTeleop(m_camera1);
  private final StopRecording c_stopRecording = new StopRecording(m_camera1);
  // and so on for however many cameras we have

  private final OI m_OI = new OI();
  // NSargent: the stuff below was already here

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //CommandScheduler.getInstance().setDefaultCommand(m_camera1, c_startRecordingAutonomous);
    //CommandScheduler.getInstance().schedule(c_startRecordingAutonomous);
    //Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // public Command getTagData(){
  //   System.out.println("XBUTTON");
  //   //return c_GetTagData;
  //   //return new GetTagData(m_camera1);
  //   return null;
  // }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // System.out.println("Configuring buttons");
    // Trigger tagButton = new Trigger(m_OI::getXButton);
    // tagButton.onTrue(getTagData());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  public Command getAutonomousCommand() {
    return c_startRecordingAutonomous;
  }

  public Command getTeleopCommand(){
    return c_startRecordingTeleop;
  }

  // NSargent: commandScheduler.getInstance.setDefaultCommand(m_camera1, getTagdata())
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getDisabledCommand() {
    // An example command will be run in autonomous
    return c_stopRecording;
  }
}
