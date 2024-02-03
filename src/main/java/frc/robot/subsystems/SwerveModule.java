// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. 
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
 *  
 * THIS CODE ONLY WORKS IF YOU SET THE CANCODERS TO BOOT TO ABSOLUTE POSITION
 * 
 * OTHERWISE THE WHEELS WILL NOT INITIALIZE IN THE CORRECT POSITIONS
 * 
 * ! ! ! ! ! NOTE ! ! ! ! ! ! !
*/

public class SwerveModule extends Diagnostics
{
    private SwerveModuleConfig cfg;
    private SwerveModuleIDConfig idcfg;
    private TalonFX steerMotor, driveMotor;
    // private SwerveModuleIDConfig ids;
    private CANcoder steerEncoder;
    public Translation2d position;
    public VelocityVoltage driveVelocityVoltage;
    public PositionVoltage steerPositionVoltage;
    
    
    /** Constructs a swerve module class. Initializes drive and steer motors
     * 
     * @param cfg swerve module configuration values for this module
     * @param ids Can Ids for this module
     */
    public SwerveModule(SwerveModuleConfig cfg, SwerveModuleIDConfig ids)
    {
        this.position = cfg.position;
        this.cfg = cfg;
        this.idcfg = ids;
        // this.ids = ids;
        steerMotor = new TalonFX(ids.steerMotorID);
        driveMotor = new TalonFX(ids.driveMotorID);
        steerEncoder = new CANcoder(ids.steerEncoderID);
        driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        steerPositionVoltage = new PositionVoltage(0).withSlot(0);
        setUpMotors();
        
    }

    public static void initPreferences() 
    {
  
    }

    //Returns diagnostics
    public String getDiagnostics() 
    {
        StatusCode error;
        String result = new String();
        // Fault faults = new Faults();
        // driveMotor.getFaults(faults);
        // if (faults.hasAnyFault()) {
        //     result += faults.toString();
        // }
        // steerMotor.getFaults(faults);
        // if(faults.hasAnyFault()){
        //     result += faults.toString();
        // }
        error = steerEncoder.clearStickyFaults(500);
        if (error != StatusCode.OK) 
        {
            result += String.format(" Module %d, steerEncoder %d, error.", cfg.moduleNumber, idcfg.steerEncoderID);
        }
        return result;
    }

    @Override
    public void runDiagnostics() 
    {
        String result = new String();
        boolean isOK = true;
        //TODO: run diagnostics here
        super.setDiagnosticResult(result);
        super.setOK(isOK);
    }

    // Populate a SwerveModulePosition object from the state of this module.
    public void updatePosition(SwerveModulePosition position)
    {
        position.angle = Rotation2d.fromRotations(getSteerRotations());
        position.distanceMeters = getDriveEncoder();
    }

    // Return steering sensor angle in rotations. 0 = dead ahead on robot.
    public double getSteerRotations()
    {
        // TODO:Check 
        steerEncoder.getAbsolutePosition().refresh();
        return ((steerEncoder.getAbsolutePosition().getValue()));
        //return (steerMotor.getPosition().getValue() / cfg.radiansPerRotation) - cfg.steerAngleOffset;
        //return steerEncoder.getPosition().getValue();
    }

    public double getMotorAngle()
    {
        return steerMotor.getPosition().getValue();
    }

    // Return drive encoder in meters.
    public double getDriveEncoder()
    {
        // TODO:Check 
        return -driveMotor.getRotorPosition().getValue() * cfg.rotationsPerMeter;
    }

    // Return drive velocity in meters/second.
    public double getDriveVelocity()
    { 
        // TODO: Find out meters per second
        // TODO:Check ^^
        return -driveMotor.getRotorVelocity().getValue() / (cfg.rotationsPerMeter);
    }

    // Returns the velocity from the motor itself in rotations
    public double getDriveRawVelocity() 
    { // TODO: Same thing
        return driveMotor.getRotorVelocity().getValue();
    }
    
    //*Wrapping code from sds example swerve library
    public void setCommand(double steerRotations, double driveVelocity){
        SmartDashboard.putNumber(String.format(" Target Steer Rotations %d", cfg.moduleNumber), steerRotations);
        SmartDashboard.putNumber(String.format(" Target Drive Velocity %d", cfg.moduleNumber), driveVelocity);
        SmartDashboard.putNumber(String.format(" Steer Rotations %d", cfg.moduleNumber), getSteerRotations());
        SmartDashboard.putNumber(String.format("Drive Velocity %d", cfg.moduleNumber), getDriveVelocity());

        //double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        // driveVelocity^

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double steerMotorError = steerRotations - getSteerRotations();
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }
        driveVelocity *= cosineScalar;

        // /* Back out the expected shimmy the drive motor will see */
        // /* Find the angular rate to determine what to back out */
        // double azimuthTurnRps = m_steerVelocity.getValue();
        // /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        // double driveRateBackOut = azimuthTurnRps * m_couplingRatioDriveRotorToCANcoder;
        // velocityToSet -= driveRateBackOut;

        setDriveVelocity(driveVelocity);
        setSteerRotations(steerRotations);
    }

    //Sets the velocity for the drive motors
    public void setDriveVelocity(double driveVelocity)
    {
        // Velocity commands are ticks per meter in 0.1 seconds... so 1/10th the ticks/second.
        //TODO: cvt driveVelocity to units specified above
        // TODO:Check ^^
        //var error = driveMotor.setControl(new VelocityDutyCycle((-driveVelocity * cfg.metersPerRotation) / 10));
        // VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
        // var error = driveMotor.setControl(velocityDutyCycle.withVelocity((-driveVelocity * cfg.metersPerRotation) / 10));
        driveMotor.setControl(driveVelocityVoltage.withVelocity((driveVelocity * cfg.rotationsPerMeter)));
        SmartDashboard.putNumber("commanded Drive Rps", (driveVelocity * cfg.rotationsPerMeter));
        //System.out.println(error.getDescription());
    }

    //setSteerAngle in radians
    public void setSteerRotations(double steerRotations)
    {
        //TODO: fix steering angle units and add offset
        // TODO:Check ^^
        //(steeringAngle + cfg.steerAngleOffset) * cfg.tickPerRadian
        //steerMotor.setControl(steerPositionVoltage.withPosition((steeringAngle +  cfg.steerAngleOffset) * cfg.radiansPerRotation));
        //steerMotor.setControl(steerPositionDutyCycle.withPosition((steeringAngle +  cfg.steerAngleOffset)));
        //steerMotor.setControl(steerPositionDutyCycle.withPosition( ste))
        steerMotor.setControl(steerPositionVoltage.withPosition(steerRotations));
        //SmartDashboard.putNumber("Commanded Steer Angle", (steeringAngle +  cfg.steerAngleOffset) * cfg.radiansPerRotation);
    }

    /**Sets motors in the module to brake or coast mode
     * 
     * @param brake a boolean to indicate if motors should be in brake mode or not
     */
    public void setBrake(boolean brake)
    {
        if(brake)
        {
            steerMotor.setNeutralMode(NeutralModeValue.Brake);
            driveMotor.setNeutralMode(NeutralModeValue.Brake);
        }
        else
        {
            steerMotor.setNeutralMode(NeutralModeValue.Coast);
            driveMotor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    // configures motors with PIDF values, if it is inverted or not, current limits
    public void setUpMotors()
    {

        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        var error = steerMotor.getConfigurator().apply(new TalonFXConfiguration());

        if (error != StatusCode.OK) 
        {
            System.out.print(String.format("Module %d STEER MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }

        error = driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        if (error != StatusCode.OK) 
        {
            System.out.println(String.format("Module %d DRIVE MOTOR ERROR: %s", cfg.moduleNumber, error.toString()));
        }



        // Set control direction of motors:
        steerMotor.setInverted(true);
        driveMotor.setInverted(false);


        // Default to brakes off:
        steerMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);

        //steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.steerCurrentLimit, cfg.steerCurrentThreshold, cfg.steerCurrentThresholdTime));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.steerCurrentLimit));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentThreshold(cfg.steerCurrentThreshold));
        steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(cfg.steerCurrentThresholdTime));

        //driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, cfg.driveCurrentLimit, cfg.driveCurrentThreshold, cfg.driveCurrentThresholdTime));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(cfg.driveCurrentLimit));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentThreshold(cfg.driveCurrentThreshold));
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(cfg.driveCurrentThresholdTime));

        //Set up talon for CAN encoder
        // error = steerMotor.configRemoteFeedbackFilter(steerEncoder, 0);
        // if(error != StatusCode.OK)
        // {
        //     System.out.println(String.format("Module %d configRemoteFeedbackFilter failed: %s ", cfg.moduleNumber, error));
        // }
        
        MagnetSensorConfigs mgSenseCfg = new MagnetSensorConfigs();
        // mgSenseCfg.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        // mgSenseCfg.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // mgSenseCfg.MagnetOffset = cfg.steerRotationOffset; 

        StatusCode response = steerEncoder.getConfigurator().refresh(mgSenseCfg, 0.1);
        System.out.println("steerEncoder response: " + response.getDescription());
        System.out.println("Magnet AbsoluteSensorRange: " + mgSenseCfg.AbsoluteSensorRange);
        System.out.println("Magnet SensorDirection: " + mgSenseCfg.SensorDirection);
        System.out.println("Magnet MagnetOffset: " + mgSenseCfg.MagnetOffset);
        // StatusCode response = steerEncoder.getConfigurator().apply(mgSenseCfg);
        // if (!response.isOK())
        // {
        //     System.out.println("TalonFX ID " + steerEncoder.getDeviceID() + " failed config with error " + response.toString());
        // }
        //steerEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf));
        //steerEncoder.getConfigurator().apply(new CANcoderConfiguration())
        //steerEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        
        // steerEncoder.getConfigurator().setPosition(steerEncoder.getAbsolutePosition().getValue());
        
        //steerMotor.setPosition(0);
        steerConfigs.Feedback.FeedbackRemoteSensorID = idcfg.steerEncoderID;
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfigs.Feedback.RotorToSensorRatio = (150.0 / 7.0);
        steerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        //steerMotor.getConfigurator().apply(new TalonFXConfiguration().Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder));
        error = steerMotor.getConfigurator().apply(steerConfigs);
        //steerMotor.setPosition(0.0);
        if(error != StatusCode.OK)
        {
            System.out.println(String.format("Module %d configSelectedFeedbackSensor failed: %s ", cfg.moduleNumber, error));
        }

        //driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.getConfigurator().apply(driveConfigs);
        driveMotor.setPosition(0);
    
        // PID Loop settings for steering position control:
        var steerMotorClosedLoopConfig = new Slot0Configs();

        steerMotorClosedLoopConfig.withKP(cfg.steerP); 
        steerMotorClosedLoopConfig.withKI(cfg.steerI); 
        steerMotorClosedLoopConfig.withKD(cfg.steerD); 
        steerMotorClosedLoopConfig.withKV(cfg.steerF); 
        
        steerMotor.getConfigurator().apply(steerMotorClosedLoopConfig, 0.05);

        
        // PID Loop settings for drive velocity control:
        var driveMotorClosedLoopConfig = new Slot0Configs();

        // driveMotorClosedLoopConfig.kP = cfg.driveP;
        // driveMotorClosedLoopConfig.kI = cfg.driveI;
        // driveMotorClosedLoopConfig.kD = cfg.driveD;
        // driveMotorClosedLoopConfig.kV = cfg.driveF;
        driveMotorClosedLoopConfig.withKP(cfg.driveP);
        driveMotorClosedLoopConfig.withKI(cfg.driveI);
        driveMotorClosedLoopConfig.withKD(cfg.driveD);
        driveMotorClosedLoopConfig.withKV(cfg.driveF);

        var driveError = driveMotor.getConfigurator().apply(driveMotorClosedLoopConfig, 0.05);
        System.out.println(driveError);

    }

    /**Sets the percent output velocity to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugTranslate(double power)
    {
        driveMotor.setControl(new DutyCycleOut(power));
    }

    /**Sets the percent output velocity of wheel angle to power
     * 
     * @param power the percentage the motor should operate at
     */
    public void setDebugRotate(double power)
    {
        steerMotor.setControl(new DutyCycleOut(power));
    }
}
