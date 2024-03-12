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
        Path.Point point2 = new Path.Point(3.0, 0.0);
        Path.Point point3 = new Path.Point(3.0, -3.0);
        point2.blend_radius = 2.0;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Path.Segment(point1, point2, 0.0, 2.0));
        segments.add(new Path.Segment(point2, point3, 0, 2.0));
        segments.get(0).width = 2.0;
        
        Path path = new Path(segments, 0);

        path.transverseVelocity = 1.5;
        

        return SchemaDriveAuto.create(new DrivePathSchema(m_drivetrain, path), m_drivetrain);
    }
}
