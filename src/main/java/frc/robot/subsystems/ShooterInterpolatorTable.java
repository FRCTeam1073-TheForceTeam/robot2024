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
        shooterVelocityTable.put(1.38, 25.0); //20
        shooterVelocityTable.put(2.73, 22.0); //23
        shooterVelocityTable.put(3.02, 23.0); //27
        shooterVelocityTable.put(3.56, 24.0); // 29.8
        shooterVelocityTable.put(4.08, 27.0); //32
        shooterVelocityTable.put(4.7, 28.0); // offset: 0
        shooterVelocityTable.put(5.0, 29.0);
        shooterVelocityTable.put(5.65, 32.0); // offset: 10 inches left
        shooterVelocityTable.put(6.17, 32.0); // offset: 11-12 inches left
        shooterVelocityTable.put(6.82, 34.0); // offset: 13.5 inches left
        // shooterVelocityTable.put(4.7, 28.0); // offset: 6 inches left
        // shooterVelocityTable.put(5.0, 29.0); // close enough
        // shooterVelocityTable.put(5.65, 32.0); // offset: 13.5 inches left
        // shooterVelocityTable.put(6.17, 32.0); // offset: 6 inches left
        // shooterVelocityTable.put(6.82, 34.0); // close enough

        // ex: shooterVelocityTable.put(0.0, 0.0);;
    }

        public void setUpPivotInterpolator(){
        pivotTable.clear();
        //first value is the range, sceond value is the pivot angle
        pivotTable.put(0.0, 0.0);
        pivotTable.put(1.38, 0.0); // offset: 0
        pivotTable.put(2.73, -0.5);
        pivotTable.put(3.02, -0.51); // offset: 10 inches left
        pivotTable.put(3.56, -0.6); // offset: 11-12 inches left
        pivotTable.put(4.08, -0.645); // offset: 13.5 inches left
        pivotTable.put(4.7, -0.69); // offset: 6 inches left
        pivotTable.put(5.0, -0.7); // close enough
        pivotTable.put(5.65, -0.7415); // offset: 13.5 inches left
        pivotTable.put(6.17, -0.755); // offset: 6 inches left
        pivotTable.put(6.82, -0.7605); // close enough
    }

    public void setUpFeederInterpolator(){
        feederVelocityTable.clear();
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
    
