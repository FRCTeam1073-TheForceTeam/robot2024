// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SwerveModuleConfig;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);

    // Configure the trigger bindings
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
  private void configureBindings() 
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  public static void initPreferences() 
  {
    System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    Drivetrain.initPreferences();
    //OI.initPreferences();
    //SwerveModule.initPreferences();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    //An example command will be run in autonomous
    return SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(7, 0, new Rotation2d(0)), 0.5, 0.5), m_drivetrain);
    //return SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(-1.0, -1.0, new Rotation2d(0)), 0.5, 0.5), m_drivetrain);
    //return new DriveToPointSchema(m_drivetrain, new Pose2d(1.0, 0, new Rotation2d(Math.PI / 2)), 0.5, 0.5);

    // ArrayList<Pose2d> drivePoints = new ArrayList<>();
    // drivePoints.clear();

    // drivePoints.add(new Pose2d(-1, 0, new Rotation2d(0)));
    //drivePoints.add(new Pose2d(1, 1, new Rotation2d(0)));
    //drivePoints.add(new Pose2d(1, 1, new Rotation2d(Math.PI / 2)));
    //drivePoints.add(new Pose2d(1, 1, new Rotation2d(0.9)));

    //return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, drivePoints, 0.5, 0.5, 0.5, 1.0), m_drivetrain);
  }
}
