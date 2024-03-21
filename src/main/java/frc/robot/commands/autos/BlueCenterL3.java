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

public class BlueCenterL3 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, 
        CollectFeedCommand collectCommand, Collector collector, CollectorArm collectorArm)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(0.9208, 0.3447);
        Path.Point wingNote7 = new Path.Point(1.5, 0.313);
        Path.Point transitionPoint = new Path.Point(0.4521, -0.6529);
        Path.Point wingNote6 = new Path.Point(1.3312, -1.0337);

        double range1 = 2.0;
        // double range2 = 2.5;
        // double range3 = 3;
        Pose2d poseShootPoint1 = new Pose2d();


        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, 0, 2.5));
        segments.add(new Segment(pathShootPoint, wingNote7, 0, 2.5));
        segments.add(new Segment(wingNote7, transitionPoint, 0.0, 2.5));
        segments.add(new Segment(transitionPoint, wingNote6, 0.0, 2.5));
        segments.add(new Segment(wingNote6, transitionPoint, 0.0, 2.5));

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
            collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter)
        );
    }    
}
