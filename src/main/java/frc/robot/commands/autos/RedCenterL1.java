package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;

public class RedCenterL1 
{
    public static Command create(Drivetrain drivetrain)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point shootPoint = new Path.Point(1.5, 0.0);

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, shootPoint, 0, 2.5));

        return new ParallelCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, new Path(segments, 0)), drivetrain)
        );
    }
}
