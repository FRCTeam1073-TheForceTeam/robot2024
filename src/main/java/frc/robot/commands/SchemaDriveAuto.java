package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public static Command create(DriveThroughTrajectorySchema trajectorySchema, Drivetrain drivetrain){
        arbiter = new SchemaArbiter(drivetrain, false, true);
        arbiter.addSchema(trajectorySchema);

        return arbiter;
    }
}
