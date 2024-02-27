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

    public ShooterInterpolatorTable(){
        shooterVelocityTable = new InterpolatingDoubleTreeMap();
        setUpShooterInterpolator();
        feederVelocityTable = new InterpolatingDoubleTreeMap();
        setUpFeederInterpolator();
    }

    public void setUpShooterInterpolator(){
       shooterVelocityTable.clear();
        //first value is the range, sceond value is the speed/velocity
        shooterVelocityTable.put(3.0, 15.0); // this one is made up
        shooterVelocityTable.put(4.0, 20.0); // this one is made up
        shooterVelocityTable.put(5.7, 22.0); //pivot angle: -0.7, feeder: 22
        shooterVelocityTable.put(7.7, 25.0); //pivot angle: -0.7, feeder: 22
        // ex: shooterVelocityTable.put(Double.valueOf(15), Double.valueOf(22));
    }

    public void setUpFeederInterpolator(){
        feederVelocityTable = new InterpolatingDoubleTreeMap();
        //first value is the range, sceond value is the speed/velocity
        feederVelocityTable.put(2.0, 20.0); //all of these are made up values
        feederVelocityTable.put(4.0, 21.0); //all of these are made up values
        feederVelocityTable.put(6.0, 22.0); //all of these are made up values
        feederVelocityTable.put(7.0, 23.0); //all of these are made up values
    }

    public double interpolateShooterVelocity(double range){
        return shooterVelocityTable.get(range);  //spits out the corresponding velocity
    }

}
    
