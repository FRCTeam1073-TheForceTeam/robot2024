package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SchemaDriveAuto 
{
    private static SchemaArbiter arbiter;

    public static Command create(DriveToPointSchema pointSchema, Drivetrain drivetrain)
    {
        arbiter = new SchemaArbiter(drivetrain, true, true);
        arbiter.addSchema(pointSchema);

        return arbiter;
    }

    public static Command create(DrivePathSchema pathSchema, Drivetrain drivetrain)
    {
        arbiter = new SchemaArbiter(drivetrain, true, true);
        arbiter.addSchema(pathSchema);

        return arbiter;
    }

    public static Command create(DrivePathSchema pathSchema, AlignSpeakerAutoSchema alignAutoSchema, Drivetrain drivetrain)
    {
        arbiter = new SchemaArbiter(drivetrain, true, true);
        arbiter.addSchema(pathSchema);
        arbiter.addSchema(alignAutoSchema);

        return arbiter;
    }

    // TODO: are there supposed to be two of these? which one wins, the one on the bottom?
    // Answer: there are different constructors for DriveToPoint and DriveThroughTrajectory 
    public static Command create(DriveThroughTrajectorySchema trajectorySchema, Drivetrain drivetrain){
        arbiter = new SchemaArbiter(drivetrain, true, true);
        arbiter.addSchema(trajectorySchema);

        return arbiter;
    }
}
