// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CollectorArm extends Diagnostics {
  
  enum POSE{
    START,
    STOW,
    COLLECT,
    SCORE
  }

  TalonFX liftMotor;
  TalonFX extendMotor;
  
  MotorFault liftMotorFault = new MotorFault(liftMotor, 0);
  MotorFault extendMotorFault = new MotorFault(extendMotor, 0);
  private double liftSpeed;
  private double extendSpeed; 

  InterpolatingTreeMap<Double,Double> aMap;
  
  //TODO: Change all of these values
  private final double liftLength = 0;
  private final double armLength = 0;
  // Approximage masses of arm segments for gravity compensation:
  private final double liftArmMass = 0; 
  private final double extendArmMass = 0; 
  private final double gravityCompensationGain = 1.25;            // Increase this to increase the overall amount of gravity compensation.
  private final double gravityCompensationLiftGain = 0.0125; // 1/80 gear ratio
  private final double gravityCompensationExtendGain = 0.025;     // 1/40 gear ratio

  private final double liftAbsoluteOffset = 0;
  private final double extendAbsoluteOffset = 0;
  private final double liftTicksPerRadian = -20.656*4*2048/(2*Math.PI);
  private final double extendTicksPerRadian = 20.656*4*2048/(2*Math.PI);
  // private final JointVelocities maxVelocities = new JointVelocities(1.5, 2.1, 1);
  // private final double maxExtendAcc = 1.5;
  private final double minLiftAngle;
  private final double maxLiftAngle; 
  
  // fill in PID values
  private double lift_kP = 0;
  private double lift_kI = 0;
  private double lift_kD = 0;
  private double lift_kF = 0;

  private double extend_kP = 0;
  private double extend_kI = 0;
  private double extend_kD = 0;
  private double extend_kF = 0;

  public class JointPositions {
    public double lift;
    public double extend;

    public JointPositions(double liftAngle, double extendLength) {
      lift = liftAngle;
      extend = extendLength;
    }

    public JointPositions() {
      lift = 0.0;
      extend = 0.0;
    }

    // public JointPositions(JointPositions source) {
    //   lift = source.lift;
    //   extend = source.extend;
    // }

    public double getLiftAngle() {
      return lift;
    }

    public double getExtendLength() {
      return extend;
    }

  }

  public class JointVelocities{
    public double lift;
    public double extend;

    public JointVelocities(double liftVel, double extendVel){
      lift = liftVel;
      extend = extendVel;
    }

    public JointVelocities(){
      lift = 0.0;
      extend = 0.0;
    }

    // public void copyFrom(JointVelocities source) {
    //   lift = source.lift;
    //   extend = source.extend;
    // }
  }

  public class JointWaypoints{
    public double lift;
    public double extend;
    public double time;

    public JointWaypoints(double lift, double extend, double time){
      this.lift = lift;
      this.extend = extend;
      this.time = time;
    }

    public JointWaypoints(JointPositions positions, double time){
      lift = positions.lift;
      extend = positions.extend;
      this.time = time;
    }
  }

    public class ArmTrajectory {
      double[] liftPostions;
      double[] extendPostions;
      double[] times;
      double finalTime;
      double startTime;
    }


  /** Creates a new Arm. */
  public CollectorArm() {
    liftMotor = new TalonFX(0); // TODO: set device id
    extendMotor = new TalonFX(0); // TODO: set device id
    liftSpeed = 0;
    extendSpeed = 0;
    setUpMotors();

    // TODO: fill out values in radians
    minLiftAngle = 0;
    maxLiftAngle = 0;
  }

  public void setUpMotors() {
    //PID loop setting for lift motor
    var liftMotorClosedLoopConfig = new Slot0Configs();

    liftMotorClosedLoopConfig.withKP(lift_kP);
    liftMotorClosedLoopConfig.withKI(lift_kI);
    liftMotorClosedLoopConfig.withKD(lift_kD);
    liftMotorClosedLoopConfig.withKV(lift_kF);

    liftMotor.getConfigurator().apply(liftMotorClosedLoopConfig);

    //PID loop setting for collect motor
    var extendMotorClosedLoopConfig = new Slot0Configs();

    extendMotorClosedLoopConfig.withKP(extend_kP);
    extendMotorClosedLoopConfig.withKI(extend_kI);
    extendMotorClosedLoopConfig.withKD(extend_kD);
    extendMotorClosedLoopConfig.withKV(extend_kF);

    extendMotor.getConfigurator().apply(extendMotorClosedLoopConfig);

  }
  public void runLiftMotor(double liftSpeed)
  {
    liftMotor.setControl(new VelocityVoltage(liftSpeed * liftTicksPerRadian));
  }

  public void runExtendMotor(double extendSpeed)
  {
    extendMotor.setControl(new VelocityVoltage(extendSpeed));
  }

   public void setLiftSpeed(double liftSpeed)
  {
    this.liftSpeed = liftSpeed;
  }

  public double getLiftSpeed()
  {
    return liftSpeed;
  }

  public void setExtendSpeed(double extendSpeed)
  {
    this.extendSpeed = extendSpeed;
  }

  public double getExtendSpeed()
  {
    return extendSpeed;
  }

  public void put(double key, double value) {
    aMap.put(key, value);
  }

  public double get(double currentAngle, double goalAngle) {
    double angle = aMap.get(goalAngle);
    if (angle == 0) {
      double ceilingKey = aMap.ceilingKey(key);
      double floorKey = aMap.floorKey(key);

      if (ceilingKey == 0 && floorKey == 0) {
        return 0;
      }
      else if (ceilingKey == 0) {
        return aMap.get(floorKey);
      }
      else if (floorKey == 0) {
        return aMap.get(ceilingKey);
      }
      double floor = aMap.get(floorKey);
      double ceiling = aMap.get(ceilingKey);

      return m_interpolator.interpolate(
          floor, ceiling, m_inverseInterpolator.inverseInterpolate(floorKey, ceilingKey, key));
    } else {
      return angle;
    }
  }


  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Lift Speed", this::getLiftSpeed, this::setLiftSpeed);
    builder.addDoubleProperty("Extend Speed", this::getExtendSpeed, this::setExtendSpeed);
    builder.addBooleanProperty("ok", this::isOK, null);
    builder.addStringProperty("diagnosticResult", this::getDiagnosticResult, null);
    extendMotor.initSendable(builder);
    liftMotor.initSendable(builder);
  }

  @Override
  public void periodic() 
  {
    runExtendMotor(extendSpeed);
    runLiftMotor(liftSpeed);
    
    // interpolator tree map member smth
  }


  @Override
  public void runDiagnostics() 
  {
    String result = "";
    setOK(true);

    if(liftMotorFault.hasFaults() || extendMotorFault.hasFaults()){
        setOK(false);
    }

    result += liftMotorFault.getFaults() + extendMotorFault.getFaults();

    setDiagnosticResult(result);
  }
}
