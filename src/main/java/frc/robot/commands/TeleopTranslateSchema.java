package frc.robot.commands;

import frc.robot.subsystems.OI;

public class TeleopTranslateSchema extends MotionSchema 
{
    OI oi;

    public TeleopTranslateSchema(OI oi)
    {
        this.oi = oi;
    }

    @Override
    public void update() {
        // TODO: set a translate based on operator input
    }
}
