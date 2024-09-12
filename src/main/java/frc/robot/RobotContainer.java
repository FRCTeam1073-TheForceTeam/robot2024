// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.Camera;
import frc.robot.commands.ArmPoseTeleop;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ClimberTeleop;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.CollectorArmTeleop;
import frc.robot.commands.CollectorOuttakeCommand;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.LaunchFeederToSpeaker;
import frc.robot.commands.SetShotsSequences;
import frc.robot.commands.SpeakerTagAllianceSearch;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.autos.AmpL1;
import frc.robot.commands.autos.AmpL2;
import frc.robot.commands.autos.AmpL3;
import frc.robot.commands.autos.AmpL4;
import frc.robot.commands.autos.CenterL1;
import frc.robot.commands.autos.CenterL2;
import frc.robot.commands.autos.CenterL3;
import frc.robot.commands.autos.CenterL4;
import frc.robot.commands.autos.LeaveAuto;
import frc.robot.commands.autos.SourceL1;
import frc.robot.commands.autos.SourceL2;
import frc.robot.commands.autos.SourceL3;
import frc.robot.commands.autos.SourceL4;
import frc.robot.commands.autos.SourceSnowplow;
import frc.robot.commands.autos.TestAuto;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModuleConfig;

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
  private final Climber m_climber = new Climber(m_OI);
  private final RangeFinder m_rangeFinder = new RangeFinder();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Headlight m_headlight = new Headlight();

  private final LaunchFeederToSpeaker m_launchFeederToSpeaker = new LaunchFeederToSpeaker();
  private final SetShotsSequences m_setShotsSequences = new SetShotsSequences();
  private final CancelCommand m_cancelCommand = new CancelCommand();
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_headlight, m_OI, m_aprilTagFinder);
  private final Collector m_collector = new Collector();
  private final CollectorArm m_collectorArm = new CollectorArm();
  private final CollectFeedCommand m_collectAndFeed = new CollectFeedCommand(m_collectorArm);
  //private final CollectorTeleop m_collectorTeleopCommand = new CollectorTeleop(m_collector, m_collectorArm, m_drivetrain, m_OI);
  private final CollectorArmTeleop m_collectorArmTeleop = new CollectorArmTeleop(m_collectorArm, m_OI);
  private final CollectorOuttakeCommand m_collectorOuttakeCommand = new CollectorOuttakeCommand(m_collector, m_collectorArm, m_drivetrain);
  private final ArmPoseTeleop m_armPoseTeleop = new ArmPoseTeleop(m_collectorArm);
  private final HandoffCommand m_handoffCommand = new HandoffCommand();
  private final ClimberTeleop m_ClimberTeleop = new ClimberTeleop(m_climber, m_OI);
  private final SpeakerTagAllianceSearch m_allianceSearch = new SpeakerTagAllianceSearch(m_aprilTagFinder);


  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kNoAuto = "No Autonomous";
  private static final String kSourceL2 = "Source L2";
  private static final String kSourceL1 = "Source L1";
  private static final String kSourceL3 = "Source L3";
  private static final String kSourceL4 = "Source L4";
  private static final String kCenterL1 = "Center L1";
  private static final String kCenterL2 = "Center L2";
  private static final String kCenterL3 = "Center L3";
  private static final String kCenterL4 = "Center L4";
  private static final String kAmpL1 = "Amp L1";
  private static final String kAmpL2 = "Amp L2";
  private static final String kAmpL3 = "Amp L3";
  private static final String kAmpL4 = "Amp L4";
  private static final String kSourceSnowplow = "Source Snowplow";
  private static final String kLeaveAuto = "Leave Auto";
  private static final String kTestAuto = "Test Auto";
  
  private boolean isRed;

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Bling m_bling = new Bling(m_collector, m_feeder, m_shooter, m_aprilTagFinder);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shooterTestCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_feeder, m_feederTestCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_collector, m_collectorTeleopCommand); 
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
    SmartDashboard.putData(m_headlight);


    m_chooser.setDefaultOption("No Autonomous", kNoAuto);

    m_chooser.addOption("Source L1", kSourceL1);
    m_chooser.addOption("Source L2", kSourceL2);
    m_chooser.addOption("Source L3", kSourceL3);
    m_chooser.addOption("Source L4", kSourceL4);
    m_chooser.addOption("Source Snowplow", kSourceSnowplow);
    m_chooser.addOption("Center L1", kCenterL1);
    m_chooser.addOption("Center L2", kCenterL2);
    m_chooser.addOption("Center L3", kCenterL3);
    m_chooser.addOption("Center L4", kCenterL4);
    m_chooser.addOption("Amp L1", kAmpL1);
    m_chooser.addOption("Amp L2", kAmpL2);
    m_chooser.addOption("Amp L3", kAmpL3);
    m_chooser.addOption("Amp L4", kAmpL4); 
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
    loadNoteToFeeder.onTrue(m_collectAndFeed.runCollectFeedCommand(m_drivetrain, m_collector, m_collectorArm, m_pivot, m_feeder, m_shooter, m_rangeFinder, m_aprilTagFinder));
    
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

    Trigger collectCommand = new Trigger(m_OI::getOperatorRightBumper);
    collectCommand.onTrue(m_collectAndFeed.runCollectCommand(m_drivetrain, m_collector, m_collectorArm));

    Trigger outtakeCommand = new Trigger(m_OI::getOperatorLeftBumper);
    outtakeCommand.onTrue(m_collectorOuttakeCommand);

    // System.out.println("Configuring buttons");
    // Trigger tagButton = new Trigger(m_OI::getXButton);
    // tagButton.onTrue(getTagData());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    Trigger dynamicAimCommand = new Trigger(m_OI::getOperatorDPadRight);
    dynamicAimCommand.onTrue(m_launchFeederToSpeaker.runDynamicAiming(m_pivot, m_shooter, m_rangeFinder, m_drivetrain, m_aprilTagFinder));
  }

  public void autonomousInit()
  {
    SmartDashboard.putString("Alliance", "None");
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
    if(DriverStation.getAlliance().isPresent())
      {
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
          SmartDashboard.putString("Alliance", "Red");
          isRed = true;
        }
        else 
        {
          SmartDashboard.putString("Alliance", "Blue");
          isRed = false;
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
      case kSourceL1:
        return SourceL1.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kSourceL2:
        return SourceL2.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kSourceL3:
        return SourceL3.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kSourceL4:
        return SourceL4.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kSourceSnowplow:
        return SourceSnowplow.create(m_drivetrain, m_shooter, m_pivot, m_feeder, isRed);
      case kCenterL1:
        return CenterL1.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kCenterL2:
        return CenterL2.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kCenterL3:
        return CenterL3.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kCenterL4:
        return CenterL4.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_collectAndFeed, m_collector, m_collectorArm, m_aprilTagFinder, m_rangeFinder, isRed);
      case kAmpL1:
        return AmpL1.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder, isRed);
      case kAmpL2:
        return AmpL2.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder, m_collector, m_collectorArm, m_collectAndFeed, isRed);
      case kAmpL3:
        return AmpL3.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder, m_collector, m_collectorArm, m_collectAndFeed, isRed);
      case kAmpL4:
        return AmpL4.create(m_drivetrain, m_headlight, m_shooter, m_pivot, m_feeder, m_aprilTagFinder, m_rangeFinder, m_collector, m_collectorArm, m_collectAndFeed, isRed);
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
