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


    public void setUpShooterInterpolator(){
        shooterVelocityTable = new InterpolatingDoubleTreeMap();
        //first value is the range, sceond value is the speed/velocity
        shooterVelocityTable.put(Double.valueOf(15), Double.valueOf(22));
        shooterVelocityTable.put(Double.valueOf(10), Double.valueOf(25));
    }

    public void setUpFeederInterpolator(){
        feederVelocityTable = new InterpolatingDoubleTreeMap();
        //first value is the range, sceond value is the speed/velocity
        feederVelocityTable.put(Double.valueOf(20), Double.valueOf(0));
        feederVelocityTable.put(Double.valueOf(15), Double.valueOf(10));
        feederVelocityTable.put(Double.valueOf(10), Double.valueOf(12));

    }

    public double interpolateShooterVelocity(double range){
        return shooterVelocityTable.get(range);  //spits out the corresponding velocity
    }
    

}
