// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.PivotTestCommand;
import frc.robot.commands.ShooterTestCommand;
import frc.robot.commands.FeederTestCommand;
import frc.robot.commands.LoadTrigger;
import frc.robot.commands.RunTrigger;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.RunShooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pivot pivot = new Pivot();
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder(); 
  private final PivotTestCommand m_pivotTestCommand = new PivotTestCommand(pivot);
  private final ShooterTestCommand m_shooterTestCommand = new ShooterTestCommand(shooter);
  private final FeederTestCommand m_feederTestCommand = new FeederTestCommand(feeder);
  private final LoadTrigger m_loadTrigger = new LoadTrigger(shooter);
  private final RunTrigger m_runTrigger = new RunTrigger();
  private final SetShooterAngle m_setShooterAngle = new SetShooterAngle(shooter, 0);
  private final RunShooter m_runShooter = new RunShooter(shooter, 0, 0, 0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);
  SequentialCommandGroup fullAuto;
  
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(pivot, m_pivotTestCommand);
    CommandScheduler.getInstance().setDefaultCommand(shooter, m_shooterTestCommand);
    CommandScheduler.getInstance().setDefaultCommand(feeder, m_feederTestCommand);
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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  public static void initPreferences() {
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
  public Command getAutonomousCommand(){
    return fullAuto = 
    new SequentialCommandGroup(m_runTrigger, m_loadTrigger, m_setShooterAngle, m_runShooter);
  }
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return null;
  // }
}
