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
        shooterVelocityTable.put(0.0, 25.0); //old:25
        shooterVelocityTable.put(2.05, 15.0); //20
        shooterVelocityTable.put(3.55, 18.0); //23
        shooterVelocityTable.put(4.99, 22.0); //27
        shooterVelocityTable.put(6.55, 24.8); // 29.8
        shooterVelocityTable.put(7.3, 27.0); //32

        // ex: shooterVelocityTable.put(0.0, 0.0);;
    }

        public void setUpPivotInterpolator(){
        //first value is the range, sceond value is the pivot angle
        pivotTable.put(0.0, 0.0); // offset: 0
        pivotTable.put(2.05, -0.3); // offset: 10 inches left
        pivotTable.put(3.55, -0.505); // offset: 11-12 inches left
        pivotTable.put(4.99, -0.718); // offset: 13.5 inches left
        pivotTable.put(6.55, -0.788); // offset: 6 inches left
        pivotTable.put(7.3, -0.82); // close enough
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
    
