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
        shooterVelocityTable.put(2.75, 20.0);
        shooterVelocityTable.put(2.93, 20.0);
        shooterVelocityTable.put(3.64, 20.0);
        shooterVelocityTable.put(4.18, 20.0);
        shooterVelocityTable.put(4.66, 24.8);
        shooterVelocityTable.put(5.03, 24.9);
        shooterVelocityTable.put(5.85, 25.0);

        shooterVelocityTable.put(2.0, 15.0); //likely 2.0, literally the same thing
        shooterVelocityTable.put(2.53, 20.0); //likely 2.49
        shooterVelocityTable.put(3.1, 20.0); //likely 3.048
        shooterVelocityTable.put(3.76, 25.0);// likely 3.63
        shooterVelocityTable.put(4.78, 20.0); // likely 4.29 for AprilTag
        shooterVelocityTable.put(5.7, 22.0); // 
        shooterVelocityTable.put(5.83, 25.0); // likely 4.8 for AprilTag
        shooterVelocityTable.put(6.0, 25.0); // likely 5.28 for AprilTag
        shooterVelocityTable.put(7.7, 25.0); // 

        // ex: shooterVelocityTable.put(0.0, 0.0);;
    }

        public void setUpPivotInterpolator(){
        //first value is the range, sceond value is the pivot angle
        pivotTable.put(2.75, -0.46);
        pivotTable.put(2.93, -0.5);
        pivotTable.put(3.64, -0.53);
        pivotTable.put(4.18, -0.58);
        pivotTable.put(4.66, -0.6395);
        pivotTable.put(5.03, -0.66);
        pivotTable.put(5.85, -0.66);
        
        pivotTable.put(2.0, -0.25);
        pivotTable.put(2.53, -0.45);
        pivotTable.put(3.1, -0.54);
        pivotTable.put(3.76, -0.602);
        pivotTable.put(4.78, -0.695);
        pivotTable.put(5.83, -0.72); 
        pivotTable.put(6.0, -0.745); 
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
    
