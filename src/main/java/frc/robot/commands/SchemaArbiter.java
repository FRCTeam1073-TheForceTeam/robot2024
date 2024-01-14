package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MotionSchema.Rotate;
import frc.robot.commands.MotionSchema.Translate;
import frc.robot.subsystems.Drivetrain;

public class SchemaArbiter extends Command
{
    ArrayList<MotionSchema> schema = new ArrayList<MotionSchema>();
    Drivetrain drivetrain;
    MotionSchema.Translate totalTranslate = new MotionSchema.Translate(0, 0, 0);
    MotionSchema.Rotate totalRotate = new MotionSchema.Rotate(0, 0);

    public SchemaArbiter(Drivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    public void addSchema(MotionSchema schema)
    {
        this.schema.add(schema);
    }

    @Override
    public void initialize()
    {
        for (int i = 0; i < schema.size(); i++)
        {
            schema.get(i).initialize();
        }
    }

    @Override
    public void execute()
    {
        // clear total
        totalTranslate.zero();
        totalRotate.zero();
        // loop over all schema
        for (int i = 0; i < schema.size(); i++)
        {
            // add them into total
            totalTranslate.accumulate(schema.get(i).getTranslate());
            totalRotate.accumulate(schema.get(i).getRotate());
        }
        // safe divide total weight
        totalTranslate.resolve();
        totalRotate.resolve();
        // send to drive subsystem
        // TODO: actually do this
    }

    @Override
    public void end(boolean interrupted)
    {
        // TODO: set drivetrain velocity to 0
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
