// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Camera;
import frc.robot.commands.GetTagData;
import frc.robot.subsystems.SerialComms;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.subsystems.OI;
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
  // The robot's subsystems and commands are defined here...
  private final SerialComms m_serial = new SerialComms(SerialPort.Port.kUSB);
  private final Camera m_camera1 = new Camera(m_serial, 1);  // camID is how SerialComms and the cameras themselves tells them apart
  //private final GetTagData c_GetTagData = new GetTagData(m_camera1);
  private final OI m_OI = new OI();
  //private final Camera m_camera2 = new Camera(m_serial, 2);
  // and so on for however many cameras we have

  // NSargent: the stuff below was already here

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CommandScheduler.getInstance().setDefaultCommand(m_camera1, getTagData());
    //CommandScheduler.getInstance().schedule(getTagData());;
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
  public Command getTagData(){
    System.out.println("XBUTTON");
    //return c_GetTagData;
    return new GetTagData(m_camera1);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    System.out.println("Configuring buttons");
    Trigger tagButton = new Trigger(m_OI::getXButton);
    tagButton.onTrue(getTagData());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  // NSargent: commandScheduler.getInstance.setDefaultCommand(m_camera1, getTagdata())
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }


}
