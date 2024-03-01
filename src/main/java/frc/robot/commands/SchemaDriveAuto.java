package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class SchemaDriveAuto 
{
    private static SchemaArbiter arbiter;

    public static Command create(DriveToPointSchema pointSchema, Drivetrain drivetrain)
    {
        arbiter = new SchemaArbiter(drivetrain, false, true);
        arbiter.addSchema(pointSchema);

        return arbiter;
    }

    // TODO: are there supposed to be two of these? which one wins, the one on the bottom?
    // Answer: there are different constructors for DriveToPoint and DriveThroughTrajectory 
    public static Command create(DriveThroughTrajectorySchema trajectorySchema, Drivetrain drivetrain){
        arbiter = new SchemaArbiter(drivetrain, false, true);
        arbiter.addSchema(trajectorySchema);

        return arbiter;
    }
}
