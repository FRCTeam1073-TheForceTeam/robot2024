package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;

public class TeleopRotateSchema extends MotionSchema 
{
    OI oi;

    public TeleopRotateSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        setRotate(oi.getDriverRotate(), 1);
    }
}
