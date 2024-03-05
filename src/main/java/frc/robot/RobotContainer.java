// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SerialPort.Port serial_port = SerialPort.Port.kUSB;

  // The robot's subsystems and commands are defined here...
  private final Collector m_collector = new Collector();
  private final CollectorArm m_collectorArm = new CollectorArm();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Feeder m_feeder = new Feeder();
  private final OI m_OI = new OI();
  private final Pivot m_pivot = new Pivot();
  private final RangeFinder m_rangeFinder = new RangeFinder();
  private final Shooter m_shooter = new Shooter();
  
  private final ArmPoseTeleop m_armPoseTeleop = new ArmPoseTeleop(m_collectorArm, m_OI);
  private final AmpShootCommand m_ampShootCommand = new AmpShootCommand();
  private final CancelCommand m_cancelCommand = new CancelCommand();
  private final CollectFeedCommand m_collectAndFeed = new CollectFeedCommand();
  private final CollectorTeleop m_collectorTeleopCommand = new CollectorTeleop(m_collector, m_collectorArm, m_drivetrain, m_OI);
  private final LaunchFeederToSpeaker m_launchFeederToSpeaker = new LaunchFeederToSpeaker();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);
  

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kNoAuto = "No Autonomous";
  // ex: private static final String auto1 = "auto 1";
  private static final String kSnowPlowAuto = "Snowplow Auto";
  private static final String kLeaveAuto = "Leave Auto";
  private static final String kTestAuto = "Test Auto";

  private final SerialComms m_serial = new SerialComms(SerialPort.Port.kUSB);
  private final Camera m_camera1 = new Camera(m_serial, "1");  // camID is how SerialComms and the cameras themselves tells them apart
  private final Camera m_camera2 = new Camera(m_serial, "2");
  private final Camera[] m_cameras = {m_camera1, m_camera2};

  private final StartRecordingAutonomous c_startRecordingAutonomous = new StartRecordingAutonomous(m_cameras);
  private final StartRecordingTeleop c_startRecordingTeleop = new StartRecordingTeleop(m_cameras);
  private final StopRecording c_stopRecording = new StopRecording(m_cameras);
  // and so on for however many cameras we have


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  { 
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_collector, m_collectorTeleopCommand);
    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData(m_collector);
    SmartDashboard.putData(m_collectorArm);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_feeder);
    SmartDashboard.putData(m_pivot);
    SmartDashboard.putData(m_rangeFinder);

    m_chooser.setDefaultOption("No Autonomous", kNoAuto);
    m_chooser.addOption("Snowplow Auto", kSnowPlowAuto);
    m_chooser.addOption("Leave Auto", kLeaveAuto);
    m_chooser.addOption("Test Auto", kTestAuto);


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
    Trigger loadNoteToFeeder = new Trigger(m_OI::getOperatorLeftTrigger);
    loadNoteToFeeder.onTrue(m_collectAndFeed.runCollectFeedCommand(m_drivetrain, m_collector, m_collectorArm, m_pivot, m_feeder, m_shooter));
    
    Trigger launchFeederToSpeaker = new Trigger(m_OI::getOperatorRightTrigger);
    launchFeederToSpeaker.onTrue(m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder, m_pivot, m_rangeFinder));

    Trigger cancelCommand = new Trigger(m_OI::getOperatorBButton);
    cancelCommand.onTrue(m_cancelCommand.cancel(m_collector, m_collectorArm, m_shooter, m_feeder, m_pivot));

    Trigger armStartCommand = new Trigger(m_OI::getOperatorAButton);
    armStartCommand.onTrue(m_armPoseTeleop.startPose());

    Trigger armStowCommand = new Trigger(m_OI::getOperatorXButton);
    armStowCommand.onTrue(m_armPoseTeleop.stowPose());

    Trigger armAmpCommand = new Trigger(m_OI::getOperatorYButton);
    armAmpCommand.onTrue(m_armPoseTeleop.ampPose());

    Trigger ampShootCommand = new Trigger(m_OI::getOperatorMenuButton);
    ampShootCommand.onTrue(m_ampShootCommand.ampShot(m_shooter, m_feeder, m_pivot));
  }

  public void printAllFalseDiagnostics(){
    boolean isDisabled = DriverStation.isDisabled();
    boolean allOK = true;
    // Set allOK to the results of the printDiagnostics method for each subsystem, separated by &&
    allOK = true
      // ex. && m_subsystem.printDiagnostics(isDisabled)
      && m_collector.printDiagnostics(isDisabled)
      && m_collectorArm.printDiagnostics(isDisabled)
      && m_shooter.printDiagnostics(isDisabled) 
      && m_pivot.printDiagnostics(isDisabled) 
      && m_feeder.printDiagnostics(isDisabled)
    ;
    //TODO: Add each subsystem
    SmartDashboard.putBoolean("Engine light", allOK);
  }

  public static void initPreferences()
  {
    System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    Drivetrain.initPreferences();
  }

  public Command testAuto()
  {
    ArrayList<Pose2d> pointList = new ArrayList<Pose2d>();
    pointList.add(new Pose2d(2.0, 0.0, new Rotation2d(0)));
    pointList.add(new Pose2d(2.0, 1.0, new Rotation2d(0)));
    pointList.add(new Pose2d(4.0, 1.0, new Rotation2d(Math.PI)));
    return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, pointList, 2.0, 2.0, 5.0), m_drivetrain);
  }

  public Command getTeleopCommand(){
    return c_startRecordingTeleop;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //TODO: addc_startRecordingAutonomous; to preexisting getAutonomousCommand
    
    switch (m_chooser.getSelected()){
      case kNoAuto:
        return null;
      case kSnowPlowAuto:
        return new SequentialCommandGroup(SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(7, 0, new Rotation2d(0)), 5, 1), m_drivetrain),
          SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(7.0, 5.5, new Rotation2d(0)), 5, 1), m_drivetrain), c_startRecordingAutonomous);
      case kLeaveAuto:
        return new SequentialCommandGroup(SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(1.5, 0.0, new Rotation2d()), 1.5, 0), m_drivetrain), c_startRecordingAutonomous);
      case kTestAuto:
        return new SequentialCommandGroup(testAuto(), c_startRecordingAutonomous);
      default:
        return null;
    }
  }

  public Command getDisabledCommand() {
    return c_stopRecording;
  }
}
