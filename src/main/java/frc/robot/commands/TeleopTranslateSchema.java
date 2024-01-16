package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopTranslateSchema extends MotionSchema 
{
    OI oi;

    public TeleopTranslateSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void update(Drivetrain drivetrain) 
    {
        setTranslate(oi.getDriverTranslateX(), oi.getDriverTranslateY(), 1);
    }
}
