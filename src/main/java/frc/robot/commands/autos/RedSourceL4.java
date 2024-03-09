package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class RedSourceL4 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.5, 0.0);
        Path.Point avoidStagePost = new Path.Point(5.7, 0.75);
        Path.Point midlineNote2 = new Path.Point(8.6, 0.0);
        Path.Point stagePoint = new Path.Point(5.368, -2.37);
        Path.Point midlineNote3 = new Path.Point(8.6, -1.57);
        Path.Point midlineNote4 = new Path.Point(8.6, -3.2);
        stagePoint.blend_radius = 0.6;

        Pose2d poseShootPoint = new Pose2d(3.5, 0.0, new Rotation2d(0.853));
        double range1 = 0.0;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, 0.853, 3.0));
        segments.add(new Segment(pathShootPoint, avoidStagePost, 0.0, 3.0));
        segments.add(new Segment(avoidStagePost, midlineNote2, 0.0, 3.0));
        segments.add(new Segment(midlineNote2, stagePoint, 0.0, 3.0));
        segments.add(new Segment(stagePoint, pathShootPoint, 0.853, 3.0));
        segments.add(new Segment(pathShootPoint, stagePoint, 0.0, 3.0));
        segments.add(new Segment(stagePoint, midlineNote3, 0.0, 3.0));
        segments.add(new Segment(midlineNote3, stagePoint, 0.0, 3.0));
        segments.add(new Segment(stagePoint, pathShootPoint, 0.853, 3.0));
        segments.add(new Segment(pathShootPoint, stagePoint, 0.0, 3.0));
        segments.add(new Segment(stagePoint, midlineNote4, 0.0, 3.0));

        return new ParallelCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, new Path(segments, 0.0)), drivetrain)//,
            // new SequentialCommandGroup(
            //     new ParallelCommandGroup(
            //         new RunShooter(shooter, range1),
            //         new PivotRangeCommand(pivot, range1)
            //     ),
            //     new WaitForPoint(drivetrain, poseShootPoint, 0.1, 0.1)
            // )
        );
    }    
}
