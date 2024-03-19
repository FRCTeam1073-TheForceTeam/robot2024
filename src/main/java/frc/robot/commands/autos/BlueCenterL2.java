package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
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
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class BlueCenterL2 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, 
        CollectFeedCommand collectCommand, Collector collector, CollectorArm collectorArm, AprilTagFinder tagFinder, RangeFinder rangeFinder)
    {
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder);

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(1.5, 0.0);
        Path.Point wingNote7 = new Path.Point(1.5, -0.43);

        double range1 = 0;
        //Pose2d poseShootPoint = new Pose2d();

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(start, pathShootPoint, 0, 3)); //TODO: ask about the orientations
        //segments1.add(new Segment(shootPoint, wingNote7, 0, 2.5));
        segments1.get(0).entryActivateValue = true;
        segments1.get(0).entryActivate = alignSchema;
        segments1.get(0).exitActivateValue = false;
        segments1.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(pathShootPoint, wingNote7, 0.0, 3.0));
        segments2.add(new Segment(wingNote7, pathShootPoint, -0.768, 3.0));

        segments2.get(1).entryActivateValue = true;
        segments2.get(1).entryActivate = alignSchema;
        segments2.get(1).exitActivateValue = false;
        segments2.get(1).exitActivate = alignSchema;

        Path path1 = new Path(segments1, -0.768);
        path1.transverseVelocity = 1.5;

        Path path2 = new Path(segments2, -0.768);

        return new SequentialCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, new Path(segments1, 0)), drivetrain),
            new ParallelCommandGroup(
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            ),
            new SetPivotCommand(pivot, 0.0),
            collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter),
            new ParallelCommandGroup(
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            )
        );
    }    
}
