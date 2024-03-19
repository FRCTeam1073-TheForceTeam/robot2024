// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autos.BlueWing4Note;
import frc.robot.commands.autos.BlueCloseMidline2Note;
import frc.robot.commands.autos.BlueCloseMidline3Note;
import frc.robot.commands.autos.BlueAmpL1;
import frc.robot.commands.autos.BlueAmpL2;
import frc.robot.commands.autos.BlueAmpL3;
import frc.robot.commands.autos.BlueAmpL4;
import frc.robot.commands.autos.BlueCenterL1;
import frc.robot.commands.autos.BlueCenterL2;
import frc.robot.commands.autos.BlueCenterL3;
import frc.robot.commands.autos.BlueCenterL4;
import frc.robot.commands.autos.BlueFarMidline3Note;
import frc.robot.commands.autos.BlueSourceL1;
import frc.robot.commands.autos.BlueSourceL2;
import frc.robot.commands.autos.BlueSourceL3;
import frc.robot.commands.autos.BlueSourceL4;
import frc.robot.commands.autos.BlueSourceSnowplow;
import frc.robot.commands.autos.LeaveAuto;
import frc.robot.commands.autos.RedAmpL1;
import frc.robot.commands.autos.RedAmpL2;
import frc.robot.commands.autos.RedAmpL3;
import frc.robot.commands.autos.RedAmpL4;
import frc.robot.commands.autos.RedCenterL1;
import frc.robot.commands.autos.RedCenterL2;
import frc.robot.commands.autos.RedCenterL3;
import frc.robot.commands.autos.RedCenterL4;
import frc.robot.commands.autos.RedCloseMidline2Note;
import frc.robot.commands.autos.RedCloseSnowPlowAuto;
import frc.robot.commands.autos.RedFar1Note;
import frc.robot.commands.autos.RedFarSnowPlowAuto;
import frc.robot.commands.autos.RedSourceL1;
import frc.robot.commands.autos.RedSourceL2;
import frc.robot.commands.autos.RedSourceL3;
import frc.robot.commands.autos.RedSourceL4;
import frc.robot.commands.autos.RedSourceSnowplow;
import frc.robot.commands.autos.TestAuto;
import frc.robot.subsystems.CollectorArm.POSE;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
// import frc.robot.subsystems.Camera;
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
  private final Pivot m_pivot = new Pivot();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder(); 
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final Climber m_climber = new Climber();
  private final RangeFinder m_rangeFinder = new RangeFinder();
  private final SerialComms m_serial = new SerialComms();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder(m_serial);


  private final CollectFeedCommand m_collectAndFeed = new CollectFeedCommand();
  private final LaunchFeederToSpeaker m_launchFeederToSpeaker = new LaunchFeederToSpeaker();
  private final SetShotsSequences m_setShotsSequences = new SetShotsSequences();
  private final CancelCommand m_cancelCommand = new CancelCommand();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder);
  private final Collector m_collector = new Collector();
  private final CollectorArm m_collectorArm = new CollectorArm();
  private final CollectorTeleop m_collectorTeleopCommand = new CollectorTeleop(m_collector, m_collectorArm, m_drivetrain, m_OI);
  private final CollectorArmTeleop m_collectorArmTeleop = new CollectorArmTeleop(m_collectorArm, m_OI);
  private final ArmPoseTeleop m_armPoseTeleop = new ArmPoseTeleop(m_collectorArm);
  private final HandoffCommand m_handoffCommand = new HandoffCommand();
  private final ClimberTeleop m_ClimberTeleop = new ClimberTeleop(m_climber, m_OI);
  private final SpeakerTagAllianceSearch m_allianceSearch = new SpeakerTagAllianceSearch(m_aprilTagFinder);



  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kNoAuto = "No Autonomous";
  private static final String kRedSourceL1 = "Red Source L1";
  private static final String kRedSourceL2 = "Red Source L2";
  private static final String kRedSourceL3 = "Red Source L3";
  private static final String kRedSourceL4 = "Red Source L4";
  private static final String kRedCenterL1 = "Red Center L1";
  private static final String kRedCenterL2 = "Red Center L2";
  private static final String kRedCenterL3 = "Red Center L3";
  private static final String kRedCenterL4 = "Red Center L4";
  private static final String kRedAmpL1 = "Red Amp L1";
  private static final String kRedAmpL2 = "Red Amp L2";
  private static final String kRedAmpL3 = "Red Amp L3";
  private static final String kRedAmpL4 = "Red Amp L4";
  private static final String kRedSourceSnowplow = "Red Source Snowplow";
  private static final String kBlueSourceL1 = "Blue Source L1";
  private static final String kBlueSourceL2 = "Blue Source L2";
  private static final String kBlueSourceL3 = "Blue Source L3";
  private static final String kBlueSourceL4 = "Blue Source L4";
  private static final String kBlueSourceSnowPlow = "Blue Source Snowplow";
  private static final String kBlueCenterL1 = "Blue Center L1";
  private static final String kBlueCenterL2 = "Blue Center L2";
  private static final String kBlueCenterL3 = "Blue Center L3";
  private static final String kBlueCenterL4 = "Blue Center L4";
  private static final String kBlueAmpL1 = "Blue Amp L1";
  private static final String kBlueAmpL2 = "Blue Amp L2";
  private static final String kBlueAmpL3 = "Blue Amp L3";
  private static final String kBlueAmpL4 = "Blue Amp L4";
  private static final String kLeaveAuto = "Leave Auto";
  private static final String kTestAuto = "Test Auto";
  

  // private final Camera m_camera1 = new Camera(m_serial, "1");  // camID is how SerialComms and the cameras themselves tells them apart
  // private final Camera m_camera2 = new Camera(m_serial, "2");
  // private final Camera[] m_cameras = {m_camera1, m_camera2};

  // private final StartRecordingAutonomous c_startRecordingAutonomous = new StartRecordingAutonomous(m_cameras);
  // private final StartRecordingTeleop c_startRecordingTeleop = new StartRecordingTeleop(m_cameras);
  // private final StopRecording c_stopRecording = new StopRecording(m_cameras);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Bling m_bling = new Bling(m_collector, m_feeder, m_shooter, m_aprilTagFinder);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shooterTestCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_feeder, m_feederTestCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_collector, m_collectorTeleopCommand); 
    CommandScheduler.getInstance().setDefaultCommand(m_climber, m_ClimberTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_collectorArmTeleop);
    //CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_armPoseTeleop);
    //CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_runShooterCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_aprilTagFinder, m_allianceSearch);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData(m_collector);
    SmartDashboard.putData(m_collectorArm);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_feeder);
    SmartDashboard.putData(m_pivot);
    SmartDashboard.putData(m_rangeFinder);
    SmartDashboard.putData(m_climber);


    m_chooser.setDefaultOption("No Autonomous", kNoAuto);

    m_chooser.addOption("Red Source L1", kRedSourceL1);
    m_chooser.addOption("Red Source L2", kRedSourceL2);
    m_chooser.addOption("Red Source L3", kRedSourceL3);
    m_chooser.addOption("Red Source L4", kRedSourceL4);
    m_chooser.addOption("Red Source Snowplow", kRedSourceSnowplow);
    m_chooser.addOption("Red Center L1", kRedCenterL1);
    m_chooser.addOption("Red Center L2", kRedCenterL2);
    m_chooser.addOption("Red Center L3", kRedCenterL3);
    m_chooser.addOption("Red Center L4", kRedCenterL4);
    m_chooser.addOption("Red Amp L1", kRedAmpL1);
    m_chooser.addOption("Red Amp L2", kRedAmpL2);
    m_chooser.addOption("Red Amp L3", kRedAmpL3);
    m_chooser.addOption("Red Amp L4", kRedAmpL4);
    m_chooser.addOption("Blue Source L1", kBlueSourceL1);
    m_chooser.addOption("Blue Source L2", kBlueSourceL2);
    m_chooser.addOption("Blue Source L3", kBlueSourceL3);
    m_chooser.addOption("Blue Source L4", kBlueSourceL4);
    m_chooser.addOption("Blue Source Snowplow", kBlueSourceSnowPlow);
    m_chooser.addOption("Blue Center L1", kBlueCenterL1);
    m_chooser.addOption("Blue Center L2", kBlueCenterL2);
    m_chooser.addOption("Blue Center L3", kBlueCenterL3);
    m_chooser.addOption("Blue Center L4", kBlueCenterL4);
    m_chooser.addOption("Blue Amp L1", kBlueAmpL1);
    m_chooser.addOption("Blue Amp L2", kBlueAmpL2);
    m_chooser.addOption("Blue Amp L3", kBlueAmpL3);
    m_chooser.addOption("Blue Amp L4", kBlueAmpL4); 
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
  private void configureBindings() // TODO: NSARGENT: is this legit? configureBindings() call up on line 82
  {
    Trigger loadNoteToFeeder = new Trigger(m_OI::getOperatorLeftTrigger);
    loadNoteToFeeder.onTrue(m_collectAndFeed.runCollectFeedCommand(m_drivetrain, m_collector, m_collectorArm, m_pivot, m_feeder, m_shooter));
    
    Trigger launchFeederToSpeaker = new Trigger(m_OI::getOperatorRightTrigger);
    launchFeederToSpeaker.onTrue(m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder, m_pivot, m_rangeFinder));

    Trigger subwooferSpinUp = new Trigger(m_OI::getOperatorDPadDown);
    subwooferSpinUp.onTrue(m_setShotsSequences.runSubwooferShot(m_pivot, m_shooter));

    Trigger podiumSpinUp = new Trigger(m_OI::getOperatorDPadLeft);
    podiumSpinUp.onTrue(m_setShotsSequences.runPodiumShot(m_pivot, m_shooter));

    Trigger farSpinUp = new Trigger(m_OI::getOperatorDPadUp);
    farSpinUp.onTrue(m_setShotsSequences.runFarShot(m_pivot, m_shooter));

    Trigger cancelCommand = new Trigger(m_OI::getOperatorYButton);
    cancelCommand.onTrue(m_cancelCommand.cancel(m_collector, m_collectorArm, m_shooter, m_feeder, m_pivot));

    Trigger armStartCommand = new Trigger(m_OI::getOperatorAButton);
    armStartCommand.onTrue(m_armPoseTeleop.startPose());

    Trigger armStowCommand = new Trigger(m_OI::getOperatorXButton);
    armStowCommand.onTrue(m_armPoseTeleop.stowPose());

    Trigger armAmpCommand = new Trigger(m_OI::getOperatorBButton);
    armAmpCommand.onTrue(m_armPoseTeleop.ampPose());

    // System.out.println("Configuring buttons");
    // Trigger tagButton = new Trigger(m_OI::getXButton);
    // tagButton.onTrue(getTagData());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Trigger armStartPoseTrigger = new Trigger(m_OI::getOperatorAButton);
    // armStartPoseTrigger.onTrue(collectorScoreCommand());

    // Trigger armAmpPoseTrigger = new Trigger(m_OI::getOperatorYButton);
    // armAmpPoseTrigger.onTrue(armAmpPoseCommand());

  }

  public void autonomousInit()
  {
    if (m_aprilTagFinder != null)
    {
      if(DriverStation.getAlliance().isPresent())
      {
        if (DriverStation.getAlliance().get() == Alliance.Blue)
        {
          m_aprilTagFinder.setSearchTagId(7);
        }
        else
        {
          m_aprilTagFinder.setSearchTagId(4);
        }
      }
    }
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
    System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    Drivetrain.initPreferences();
    //OI.initPreferences();
    //SwerveModule.initPreferences();
  }

  public Command getTeleopCommand()
  {
    // return c_startRecordingTeleop;
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    //TODO: addc_startRecordingAutonomous; to preexisting getAutonomousCommand
    
    switch (m_chooser.getSelected())
    {
      case kNoAuto:
        return null;
      case kRedSourceL1:
        return RedSourceL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder);
      case kRedSourceL2:
        return RedSourceL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder);
      case kRedSourceL3:
        return RedSourceL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedSourceL4:
        return RedSourceL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedSourceSnowplow:
        return RedSourceSnowplow.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kRedCenterL1:
        return RedCenterL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kRedCenterL2:
        return RedCenterL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedCenterL3:
        return RedCenterL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedCenterL4:
        return RedCenterL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedAmpL1:
        return RedAmpL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kRedAmpL2:
        return RedAmpL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedAmpL3:
        return RedAmpL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kRedAmpL4:
        return RedAmpL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueSourceL1:
        return BlueSourceL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder);
      case kBlueSourceL2:
        return BlueSourceL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder);
      case kBlueSourceL3:
        return BlueSourceL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueSourceL4:
        return BlueSourceL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueSourceSnowPlow:
        return BlueSourceSnowplow.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kBlueCenterL1:
        return BlueCenterL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kBlueCenterL2:
        return BlueCenterL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueCenterL3:
        return BlueCenterL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueCenterL4:
        return BlueCenterL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueAmpL1:
        return BlueAmpL1.create(m_drivetrain, m_shooter, m_pivot, m_feeder);
      case kBlueAmpL2:
        return BlueAmpL2.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm); 
      case kBlueAmpL3:
        return BlueAmpL3.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kBlueAmpL4:
        return BlueAmpL4.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      case kLeaveAuto:
        return LeaveAuto.create(m_drivetrain);
      case kTestAuto:
        return TestAuto.create(m_drivetrain, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm);
      default:
        return null;
    }
  }

  public Command getDisabledCommand() {
    return null;
  }

  public Command launchFeederToSpeaker(){
    return m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder, m_pivot, m_rangeFinder);
  }
}
