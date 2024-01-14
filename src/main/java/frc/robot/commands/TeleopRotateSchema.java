package frc.robot.commands;

import frc.robot.subsystems.OI;

public class TeleopRotateSchema extends MotionSchema 
{
    OI oi;

    public TeleopRotateSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void update()
    {
        // TODO: set a rotate based on operator input
    }
}
