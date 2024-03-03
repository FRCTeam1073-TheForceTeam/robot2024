package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;

public class TestAuto 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        Path.Point point1 = new Path.Point(0.0, 0.0);
        Path.Point point2 = new Path.Point(1.0, 0.0);
        Path.Point point3 = new Path.Point(1.0, -1.0);
        Path.Point point4 = new Path.Point(2.0, -0.5);
        Path.Point point5 = new Path.Point(5.0, 0.0);

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Path.Segment(point1, point2, 0.0, 2.0));
        segments.add(new Path.Segment(point2, point3, Math.PI / 2, 2.0));
        segments.add(new Path.Segment(point3, point4, Math.PI, 2.0));
        segments.add(new Path.Segment(point4, point5, Math.PI / 4, 2.0));
        segments.add(new Path.Segment(point5, point1, 0.0, 2.0));

        return SchemaDriveAuto.create(new DrivePathSchema(m_drivetrain, new Path(segments, 0)), m_drivetrain);
    }
}
