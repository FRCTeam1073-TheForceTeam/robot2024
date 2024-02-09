package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopHeadingHoldSchema extends MotionSchema
{
    OI oi;
    boolean holdHeading;
    double heldHeading;
    // unless i'm seriously misunderstanding something, the variable is called the opposite of what it does, 
    // but I'm too scared to change it
    
    public TeleopHeadingHoldSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void initialize(Drivetrain drivetrain)
    {
        holdHeading = false;
        heldHeading = drivetrain.getHeading();
        System.out.println(heldHeading);
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
            holdHeading = false; 
            setRotate(heldHeading - drivetrain.getHeading(), 1);
        }
        else 
        {
            setRotate(0, 0);
            holdHeading = true;
        }
        SmartDashboard.putBoolean("Hold Heading ", holdHeading);
        SmartDashboard.putNumber("Held Heading ", heldHeading);
    }
}
