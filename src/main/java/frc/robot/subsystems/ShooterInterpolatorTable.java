// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterInterpolatorTable {

    private InterpolatingDoubleTreeMap shooterVelocityTable;
    private InterpolatingDoubleTreeMap feederVelocityTable;
    private InterpolatingDoubleTreeMap pivotTable;

    public ShooterInterpolatorTable(){
        shooterVelocityTable = new InterpolatingDoubleTreeMap();
        setUpShooterInterpolator();
        feederVelocityTable = new InterpolatingDoubleTreeMap();
        setUpFeederInterpolator();
        pivotTable = new InterpolatingDoubleTreeMap();
        setUpPivotInterpolator();
    }

    public void setUpShooterInterpolator(){
       shooterVelocityTable.clear();
        //first value is the range, sceond value is the speed/velocity
        shooterVelocityTable.put(2.11, 15.0);
        shooterVelocityTable.put(2.75, 20.0);
        shooterVelocityTable.put(2.93, 20.0);
        shooterVelocityTable.put(3.64, 20.0);
        shooterVelocityTable.put(4.18, 20.0);
        shooterVelocityTable.put(4.66, 24.8);
        shooterVelocityTable.put(5.03, 24.9);
        shooterVelocityTable.put(5.85, 25.0);
        shooterVelocityTable.put(6.61, 25.1);

        // ex: shooterVelocityTable.put(0.0, 0.0);;
    }

        public void setUpPivotInterpolator(){
        //first value is the range, sceond value is the pivot angle
        pivotTable.put(2.11, -0.3);
        pivotTable.put(2.75, -0.46);
        pivotTable.put(2.93, -0.5);
        pivotTable.put(3.64, -0.578);
        pivotTable.put(4.18, -0.63);
        pivotTable.put(4.66, -0.7);
        pivotTable.put(5.03, -0.718);
        pivotTable.put(5.85, -0.747);
        pivotTable.put(6.61, -0.765); // not finalized
    }

    public void setUpFeederInterpolator(){
        //first value is the range, sceond value is the speed/velocity
        feederVelocityTable.put(2.0, 25.0); 
        feederVelocityTable.put(2.53, 25.0); 
        feederVelocityTable.put(3.1, 25.0); 
        feederVelocityTable.put(3.76, 25.0); 
        feederVelocityTable.put(4.78, 25.0); 
        feederVelocityTable.put(5.83, 25.0); 
        feederVelocityTable.put(6.0, 25.0); 
    }


    public double interpolatePivotAngle(double range) {
        return pivotTable.get(range);
    }

    public double interpolateShooterVelocity(double range){
        return shooterVelocityTable.get(range);  //spits out the corresponding velocity
    }

}
    
