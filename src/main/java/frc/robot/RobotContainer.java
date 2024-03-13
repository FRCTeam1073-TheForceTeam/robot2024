// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GetAprilTagInfo;
import frc.robot.commands.CollectorTeleop;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.ArmPoseTeleop;
import frc.robot.commands.CollectorArmTeleop;
import frc.robot.commands.CollectorIntakeCommand;
import frc.robot.commands.CollectorIntakeOutCommand;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Camera;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.autos.BlueWing4Note;
import frc.robot.commands.autos.BlueCloseMidline2Note;
import frc.robot.commands.autos.BlueCloseMidline3Note;
import frc.robot.commands.autos.BlueAmpL2;
import frc.robot.commands.autos.BlueFarMidline3Note;
import frc.robot.commands.autos.BlueSourceL1;
import frc.robot.commands.autos.BlueSourceL2;
import frc.robot.commands.autos.BlueSourceL3;
import frc.robot.commands.autos.BlueSourceL4;
import frc.robot.commands.autos.LeaveAuto;
import frc.robot.commands.autos.RedCenterL1;
import frc.robot.commands.autos.RedCenterL2;
import frc.robot.commands.autos.RedCloseMidline2Note;
import frc.robot.commands.autos.RedCloseSnowPlowAuto;
import frc.robot.commands.autos.RedFar1Note;
import frc.robot.commands.autos.RedFarSnowPlowAuto;
import frc.robot.commands.autos.RedSourceL1;
import frc.robot.commands.autos.RedSourceL2;
import frc.robot.commands.autos.RedSourceL3;
import frc.robot.commands.autos.RedSourceL4;
import frc.robot.commands.autos.TestAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SerialComms;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.commands.SetAprilTagID;

import frc.robot.subsystems.CollectorArm.POSE;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.StartRecordingAutonomous;
import frc.robot.commands.StartRecordingTeleop;
import frc.robot.commands.GetAprilTagInfo;
import frc.robot.commands.StopRecording;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.SerialComms;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // SerialPort.Port serial_port = SerialPort.Port.kUSB;

  // The robot's subsystems and commands are defined here...
  // private final Pivot m_pivot = new Pivot();
  // private final Shooter m_shooter = new Shooter();
  // private final Feeder m_feeder = new Feeder(); 
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();

  //private final RangeFinder m_rangeFinder = new RangeFinder();
  // private final CollectFeedCommand m_collectAndFeed = new CollectFeedCommand();
  // private final LaunchFeederToSpeaker m_launchFeederToSpeaker = new LaunchFeederToSpeaker();
  // private final CancelCommand m_cancelCommand = new CancelCommand();
  // private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);
  // private final Collector m_collector = new Collector();
  // private final CollectorArm m_collectorArm = new CollectorArm();
  // private final CollectorTeleop m_collectorTeleopCommand = new CollectorTeleop(m_collector, m_collectorArm, m_drivetrain, m_OI);
  // private final CollectorArmTeleop m_collectorArmTeleop = new CollectorArmTeleop(m_collectorArm, m_OI);
  // private final ArmPoseTeleop m_armPoseTeleop = new ArmPoseTeleop(m_collectorArm, m_OI);
  // private final AmpShootCommand m_ampShootCommand = new AmpShootCommand();


  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // private static final String kNoAuto = "No Autonomous";
  // private static final String kRedCloseSnowPlowAuto = "Red Close Snowplow Auto";
  // private static final String kRedFarSnowPlowAuto = "Red Far Snowplow Auto";
  // private static final String kRedFar1Note = "Red Far 1 Note";
  // private static final String kBlueClose4Note = "Blue Close 4 Note";
  // private static final String kRedCloseMidline2Note = "Red Close Midline 2 Note";
  // private static final String kBlueCloseMidline2Note = "Blue Close Midline 2 Note";
  // private static final String kBlueCloseMidline3Note = "Blue Close Midline 3 Note";
  // private static final String kBlueFarMidline2Note = "Blue Far Midline 2 Note";
  // private static final String kBlueFarMidline3Note = "Blue Far Midline 3 Note";
  // private static final String kLeaveAuto = "Leave Auto";
  // private static final String kTestAuto = "Test Auto";

  private final SerialComms m_serial = new SerialComms();
  // private final Camera m_camera1 = new Camera(m_serial, "1");  // camID is how SerialComms and the cameras themselves tells them apart
  //private final Camera m_camera2 = new Camera(m_serial, "2");
  //public final Camera[] m_cameras = {m_camera1, m_camera2};
  // public final Camera[] m_cameras = {m_camera1};
  public final AprilTagFinder m_aprilTagFinder = new AprilTagFinder(m_serial);

  // private final StartRecordingAutonomous c_startRecordingAutonomous = new StartRecordingAutonomous(m_cameras);
  // private final StartRecordingTeleop c_startRecordingTeleop = new StartRecordingTeleop(m_cameras);
  // private final StopRecording c_stopRecording = new StopRecording(m_cameras);
  //private final GetAprilTagInfo c_getAprilTagInfo = new GetAprilTagInfo(m_serial, m_camera2, "1 or whatever");


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // private final Bling m_bling = new Bling();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shooterTestCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_feeder, m_feederTestCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_collector, m_collectorTeleopCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_collectorArmTeleop);
    //CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_armPoseTeleop);
    //CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_runShooterCommand);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    // SmartDashboard.putData(m_collector);
    // SmartDashboard.putData(m_collectorArm);
    // SmartDashboard.putData(m_shooter);
    // SmartDashboard.putData(m_feeder);
    // SmartDashboard.putData(m_pivot);
    // SmartDashboard.putData(m_rangeFinder);

    // m_chooser.setDefaultOption("No Autonomous", kNoAuto);
    // m_chooser.addOption("Red Close Snowplow Auto", kRedCloseSnowPlowAuto);
    // m_chooser.addOption("Red Far Snowplow Auto", kRedFarSnowPlowAuto);
    // m_chooser.addOption("Red Far 1 Note", kRedFar1Note);
    // m_chooser.addOption("Blue Close 4 Note", kBlueClose4Note);
    // m_chooser.addOption("Red Close Midline 2 Note", kRedCloseMidline2Note);
    // m_chooser.addOption("Blue Close Midline 2 Note", kBlueCloseMidline2Note);
    // m_chooser.addOption("Blue Close Midline 3 Note", kBlueCloseMidline3Note);
    // m_chooser.addOption("Blue Far Midline 2 Note", kBlueFarMidline2Note);
    // m_chooser.addOption("Blue Far Midline 3 Note", kBlueFarMidline3Note);
    // m_chooser.addOption("Leave Auto", kLeaveAuto);
    // m_chooser.addOption("Test Auto", kTestAuto);
    SmartDashboard.putData("Auto Chooser", m_chooser);

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
    // Trigger loadNoteToFeeder = new Trigger(m_OI::getOperatorLeftTrigger);
    // loadNoteToFeeder.onTrue(m_collectAndFeed.runCollectFeedCommand(m_drivetrain, m_collector, m_collectorArm, m_pivot, m_feeder, m_shooter));
    
    // Trigger launchFeederToSpeaker = new Trigger(m_OI::getOperatorRightTrigger);
    // launchFeederToSpeaker.onTrue(m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder, m_pivot, m_rangeFinder));

    // Trigger cancelCommand = new Trigger(m_OI::getOperatorBButton);
    // cancelCommand.onTrue(m_cancelCommand.cancel(m_collector, m_collectorArm, m_shooter, m_feeder, m_pivot));

    // Trigger armStartCommand = new Trigger(m_OI::getOperatorAButton);
    // armStartCommand.onTrue(m_armPoseTeleop.startPose());

    // Trigger armStowCommand = new Trigger(m_OI::getOperatorXButton);
    // armStowCommand.onTrue(m_armPoseTeleop.stowPose());

    // Trigger armAmpCommand = new Trigger(m_OI::getOperatorYButton);
    // armAmpCommand.onTrue(m_armPoseTeleop.ampPose());

    // Trigger ampShootCommand = new Trigger(m_OI::getOperatorMenuButton);
    // ampShootCommand.onTrue(m_ampShootCommand.ampShot(m_shooter, m_feeder, m_pivot));

    // tagButton.onTrue(new GetAprilTagInfo(m_serial, m_camera1, "4"));
    // Trigger tagButton1 = new Trigger(m_OI::getXButtonDriver);
    // tagButton1.onTrue(new SetAprilTagID(m_aprilTagFinder, "1"));

    // Trigger tagButton4 = new Trigger(m_OI::getYButtonDriver);
    // tagButton4.onTrue(new SetAprilTagID(m_aprilTagFinder, "4"));


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Trigger armStartPoseTrigger = new Trigger(m_OI::getOperatorAButton);
    // armStartPoseTrigger.onTrue(collectorScoreCommand());

    // Trigger armAmpPoseTrigger = new Trigger(m_OI::getOperatorYButton);
    // armAmpPoseTrigger.onTrue(armAmpPoseCommand());

  }

  public void printAllFalseDiagnostics(){
    boolean isDisabled = DriverStation.isDisabled();
    boolean allOK = true;
    // Set allOK to the results of the printDiagnostics method for each subsystem, separated by &&
    allOK = true
      // ex. && m_subsystem.printDiagnostics(isDisabled)
      // && m_collector.printDiagnostics(isDisabled)
      // && m_collectorArm.printDiagnostics(isDisabled)
      // && m_shooter.printDiagnostics(isDisabled) 
      // && m_pivot.printDiagnostics(isDisabled) 
      // && m_feeder.printDiagnostics(isDisabled)
    ;
    //TODO: Add each subsystem
    SmartDashboard.putBoolean("Engine light", allOK);
  }

  public static void initPreferences()
  {
    //System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    Drivetrain.initPreferences();
    //OI.initPreferences();
    //SwerveModule.initPreferences();
  }

  public Command getTeleopCommand() {
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
  //   switch (m_chooser.getSelected())
  //   {
  //     case kNoAuto:
  //       return null;
  //     case kRedCloseSnowPlowAuto:
  //       return RedCloseSnowPlowAuto.create(m_drivetrain);
  //     case kRedFarSnowPlowAuto:
  //       return RedFarSnowPlowAuto.create(m_drivetrain);
  //     case kRedFar1Note:
  //       return RedFar1Note.create(m_drivetrain);
  //     case kBlueClose4Note:
  //       return BlueClose4Note.create(m_drivetrain);
  //     case kRedCloseMidline2Note:
  //       return RedCloseMidline2Note.create(m_drivetrain);
  //     case kBlueCloseMidline2Note:
  //       return BlueCloseMidline2Note.create(m_drivetrain);
  //     case kBlueCloseMidline3Note:
  //       return BlueCloseMidline3Note.create(m_drivetrain);
  //     case kBlueFarMidline2Note:
  //       return BlueFarMidline2Note.create(m_drivetrain);
  //     case kBlueFarMidline3Note:
  //       return BlueFarMidline3Note.create(m_drivetrain);
  //     case kLeaveAuto:
  //       return LeaveAuto.create(m_drivetrain);
  //     case kTestAuto:
  //       return TestAuto.create(m_drivetrain);
  //     default:
  //       return null;
  //   }
  // }

  public Command getDisabledCommand() {
    return null;
  }

  // public Command launchFeederToSpeaker(){
  //   return m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder, m_pivot, m_rangeFinder);
  // }
}
