package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.StopShooter;
import frc.robot.commands.WaitForPoint;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class BlueCenterL4 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, 
        CollectFeedCommand collectCommand, Collector collector, CollectorArm collectorArm)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point shootPoint = new Path.Point(1.5, 0.0);
        Path.Point wingNote7 = new Path.Point(1.5, -0.43);
        Path.Point avoidStage = new Path.Point();
        Path.Point midlineNote4 = new Path.Point();

        double range1 = 0;
        double range2 = 0;
        Pose2d poseShootPoint1 = new Pose2d();
        Pose2d poseShootPoint2 = new Pose2d();

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, shootPoint, 0, 2.5));
        segments.add(new Segment(shootPoint, wingNote7, 0, 2.5));
        segments.add(new Segment(wingNote7, avoidStage, 0.0, 2.5));
        segments.add(new Segment(avoidStage, midlineNote4, 0.0, 2.5));
        segments.add(new Segment(midlineNote4, avoidStage, 0.0, 2.5));

        return new ParallelCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, new Path(segments, 0)), drivetrain),
            new ParallelCommandGroup(
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new WaitForPoint(drivetrain, poseShootPoint1, 0.25, 0.15),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            ),
            new SetPivotCommand(pivot, 0.0),
            collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter),
            new ParallelCommandGroup(
                new RunShooter(shooter, range2),
                new PivotRangeCommand(pivot, range2)
            ),
            new WaitForPoint(drivetrain, poseShootPoint2, 0.25, 0.15),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            )
        );
    }     
}
