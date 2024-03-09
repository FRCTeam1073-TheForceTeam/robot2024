package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class Turn180Schema extends MotionSchema
{
    OI oi;
    double targetHeading;
    double robotHeading;
    
    public Turn180Schema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void initialize(Drivetrain drivetrain)
    {
        
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        robotHeading = drivetrain.getHeadingRadians();
        if (oi.getDriverRawButton(8)) // TODO: set this to an actual button
        {
            targetHeading = drivetrain.getHeadingRadians() + Math.PI;
        }

        setRotate(targetHeading - robotHeading, 1.0);
    }
}
