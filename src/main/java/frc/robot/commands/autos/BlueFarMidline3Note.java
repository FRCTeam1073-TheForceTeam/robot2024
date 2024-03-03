package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class BlueFarMidline3Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        Path.Point point1 = new Path.Point(0.0, 0.0);
        Path.Point point2 = new Path.Point(1.25, 1.1);
        Path.Point point3 = new Path.Point(6.7, -1.2);
        Path.Point point4 = new Path.Point(3.48, -0.47);
        Path.Point point5 = new Path.Point(6.3, 0.67);

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(point1, point2, -Math.PI / 6, 2.5));
        segments.add(new Segment(point2, point3, 0.0, 2.5));
        segments.add(new Segment(point3, point4, 0.43, 2.5));
        segments.add(new Segment(point4, point5, 0, 2.5));
        segments.add(new Segment(point5, point4, 0.43, 2.5));

        return SchemaDriveAuto.create(new DrivePathSchema(m_drivetrain, new Path(segments, 0.43)), m_drivetrain);
    }
    
}
