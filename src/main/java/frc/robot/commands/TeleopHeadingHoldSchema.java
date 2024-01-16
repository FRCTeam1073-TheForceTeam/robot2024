package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopHeadingHoldSchema extends MotionSchema
{
    OI oi;
    boolean holdHeading;
    double heldHeading;
    
    public TeleopHeadingHoldSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void initialize(Drivetrain drivetrain)
    {
        holdHeading = true;
        heldHeading = 0.0;
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        if (!holdHeading)
        {
            heldHeading = drivetrain.getHeading();
        }
        if (oi.getDriverRotate() == 0)
        {
            setRotate(heldHeading - drivetrain.getHeading(), 1);
        }
        else 
        {
            setRotate(0, 0);
            holdHeading = false;
        }
    }
}
