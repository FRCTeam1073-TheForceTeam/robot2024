package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;

public class BlueCloseMidline2Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        Path.Point point1 = new Path.Point(0.0, 0.0);
        Path.Point point2 = new Path.Point(1.19, -0.97);
        Path.Point point3 = new Path.Point(7.37, 0.1);
        Path.Point point4 = new Path.Point(5.0, -0.14);

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(point1, point2, 0.44, 2.5));
        segments.add(new Segment(point2, point3, 0, 2.5));
        segments.add(new Segment(point3, point4, 0.33, 2.5));

        return SchemaDriveAuto.create(new DrivePathSchema(m_drivetrain, new Path(segments, 0.33)), m_drivetrain);
    }
    
}
