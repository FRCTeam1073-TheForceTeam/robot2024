package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    boolean isFieldCentric;
    boolean stopOnEnd;

    public SchemaArbiter(Drivetrain drivetrain, boolean stopOnEnd)
    {
        this.drivetrain = drivetrain;
        isFieldCentric = true;
        this.stopOnEnd = stopOnEnd;
        addRequirements(drivetrain);
    }

    public void setFieldCentric(boolean fieldCentric)
    {
        isFieldCentric = fieldCentric;
    }

    public boolean getFieldCentric()
    {
        return isFieldCentric;
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
            schema.get(i).initialize(drivetrain);
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
        if (isFieldCentric)
        {
            drivetrain.setTargetChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    totalTranslate.vx, 
                    totalTranslate.vy,
                    totalRotate.omega, 
                    Rotation2d.fromDegrees(drivetrain.getHeadingDegrees()) // gets fused heading
                )
            );
        }
        else
        {
            drivetrain.setTargetChassisSpeeds(
                new ChassisSpeeds(
                    totalTranslate.vx, 
                    totalTranslate.vy, 
                    totalRotate.omega
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        if (stopOnEnd)
        {
            drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
