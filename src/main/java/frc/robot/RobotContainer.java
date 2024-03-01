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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SerialComms;
import frc.robot.subsystems.SwerveModuleConfig;
import frc.robot.subsystems.CollectorArm.POSE;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;

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
  SerialPort.Port serial_port = SerialPort.Port.kUSB;

  // The robot's subsystems and commands are defined here...
  private final Pivot m_pivot = new Pivot();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder(); 
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final RangeFinder m_rangeFinder = new RangeFinder();
  private final LaunchFeederToSpeaker m_launchFeederToSpeaker = new LaunchFeederToSpeaker();
  
  private final PivotTestCommand m_pivotTestCommand = new PivotTestCommand(m_pivot);
  // private final ShooterTestCommand m_shooterTestCommand = new ShooterTestCommand(m_shooter, m_OI);
  // private final FeederTestCommand m_feederTestCommand = new FeederTestCommand(m_feeder, m_OI);
  // private final LoadFeeder loadFeeder = new LoadFeeder(m_feeder);
  // private final RunFeeder runFeeder = new RunFeeder(m_feeder);
  // private final SetShooterAngle setShooterAngle = new SetShooterAngle(m_feeder, 0);
  // private final RunShooter runShooter = new RunShooter(m_shooter, 0, 0, 0);

  private final StopShooter m_stopShooter = new StopShooter(m_shooter);
  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI);
  private final Collector m_collector = new Collector();
  private final CollectorArm m_collectorArm = new CollectorArm();
  private final CollectorTeleop m_collectorTeleopCommand = new CollectorTeleop(m_collector, m_collectorArm, m_drivetrain, m_OI);
  private final CollectorArmTeleop m_collectorArmTeleop = new CollectorArmTeleop(m_collectorArm, m_OI);
  private final ArmPoseTeleop m_armPoseTeleop = new ArmPoseTeleop(m_collectorArm, m_OI);


  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kNoAuto = "No Autonomous";
  // ex: private static final String auto1 = "auto 1";
  private static final String kSnowPlowAuto = "Snowplow Auto";
  private static final String kLeaveAuto = "Leave Auto";
  private static final String kTestAuto = "Test Auto";

  private final SerialComms m_serial = new SerialComms(SerialPort.Port.kUSB);
  private final Camera m_camera1 = new Camera(m_serial, "1");  // camID is how SerialComms and the cameras themselves tells them apart
  private final Camera m_camera2 = new Camera(m_serial, "2");
  public final Camera[] m_cameras = {m_camera1, m_camera2};

  private final StartRecordingAutonomous c_startRecordingAutonomous = new StartRecordingAutonomous(m_cameras);
  private final StartRecordingTeleop c_startRecordingTeleop = new StartRecordingTeleop(m_cameras);
  private final StopRecording c_stopRecording = new StopRecording(m_cameras);
  private final GetAprilTagInfo c_getAprilTagInfo = new GetAprilTagInfo(m_serial, m_camera1);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Bling m_bling = new Bling();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  { CommandScheduler.getInstance().setDefaultCommand(m_pivot, m_pivotTestCommand);
    // CommandScheduler.getInstance().setDefaultCommand(m_shooter, m_shooterTestCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_feeder, m_feederTestCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_collector, m_collectorTeleopCommand);
    //CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_collectorArmTeleop);
    //CommandScheduler.getInstance().setDefaultCommand(m_collectorArm, m_armPoseTeleop);
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
  private void configureBindings() // TODO: NSARGENT: is this legit? configureBindings() call up on line 82
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //Trigger.getOperatorRawButton1.toggleOnTrue(loadNoteToFeeder());
    
    Trigger loadNoteToFeeder = new Trigger(m_OI::getOperatorLeftTrigger);
    loadNoteToFeeder.onTrue(new LoadFeeder(m_feeder));
    
    Trigger launchFeederToSpeaker = new Trigger(m_OI::getOperatorRightTrigger);
    launchFeederToSpeaker.onTrue(m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder));

    // System.out.println("Configuring buttons");
    Trigger tagButton = new Trigger(m_OI::getXButtonDriver);
    tagButton.onTrue(c_getAprilTagInfo);
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    Trigger armStartPoseTrigger = new Trigger(m_OI::getOperatorAButton);
    armStartPoseTrigger.onTrue(collectorScoreCommand());

    Trigger armAmpPoseTrigger = new Trigger(m_OI::getOperatorYButton);
    armAmpPoseTrigger.onTrue(armAmpPoseCommand());

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
    //System.out.println("RobotContainer: init Preferences.");
    SwerveModuleConfig.initPreferences();
    Drivetrain.initPreferences();
    //OI.initPreferences();
    //SwerveModule.initPreferences();
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
    return new SequentialCommandGroup(c_startRecordingTeleop, c_getAprilTagInfo);
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
    // An example command will be run in autonomous

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

  public Command getDisabledCommand() {
    return c_stopRecording;
  }

  public Command launchFeederToSpeaker(){
    return m_launchFeederToSpeaker.runLaunchFeedertoSpeaker(m_shooter, m_feeder);
    // return new SequentialCommandGroup(
    //   new RunShooter(m_shooter, 7.7), //, m_rangeFinder.getRange()),
    //  new ParallelRaceGroup(
    //    new RunFeeder(m_feeder, 30), 
    //    new WaitCommand(1)),
    //   new StopShooter(m_shooter),
    //   new RunFeeder(m_feeder, 0)
    // );
  }

  public Command armStartPoseCommand(){
    return new ArmPoseCommand(m_collectorArm, POSE.START, false);
  }

  public Command armAmpPoseCommand(){
    return new ArmPoseCommand(m_collectorArm, POSE.AMP, false);
  }

  public Command armTestCommand(){
    return new SequentialCommandGroup(
      new ArmPoseCommand(m_collectorArm, POSE.AMP, false),
      new WaitCommand(2),
      new ArmPoseCommand(m_collectorArm, POSE.START, false)
    );
  }

  public Command collectorScoreCommand(){
    return new SequentialCommandGroup(
      new CollectorIntakeCommand(m_collector, m_collectorArm, m_drivetrain),
      new ArmPoseCommand(m_collectorArm, POSE.AMP, false),
      new CollectorIntakeOutCommand(m_collector, m_collectorArm, m_drivetrain),
      new ArmPoseCommand(m_collectorArm, POSE.START, false)
    );
  }
}
