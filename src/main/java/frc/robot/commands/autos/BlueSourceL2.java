package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.NWSetPivot;
import frc.robot.commands.NWStopShooter;
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
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class BlueSourceL2 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, 
        CollectFeedCommand collectCommand, Collector collector, CollectorArm collectorArm, AprilTagFinder tagFinder, RangeFinder rangeFinder)
    {
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder, headlight);

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.165, 0.848);
        Path.Point avoidStagePost = new Path.Point(5.7, -0.75);
        Path.Point midlineNote2 = new Path.Point(8.1, 0.35);
        Path.Point stagePoint = new Path.Point(5.368, 2.37);
        stagePoint.blend_radius = 1.0;
        // avoidStagePost.blend_radius = 0.6;

        double range1 = 3.9;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(start, pathShootPoint, -Math.PI / 6, 2.5));

        segments1.get(0).entryActivateValue = true;
        segments1.get(0).entryActivate = alignSchema;
        segments1.get(0).exitActivateValue = false;
        segments1.get(0).exitActivate = alignSchema;
        

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(pathShootPoint, avoidStagePost, 0.0, 2.5));
        segments2.add(new Segment(avoidStagePost, midlineNote2, 0.0, 2.5));
        segments2.add(new Segment(midlineNote2, avoidStagePost, 0.0, 2.5));
        segments2.add(new Segment(avoidStagePost, pathShootPoint, -Math.PI / 6, 2.5));

        segments2.get(3).entryActivateValue = true;
        segments2.get(3).entryActivate = alignSchema;
        segments2.get(3).exitActivateValue = false;
        segments2.get(3).exitActivate = alignSchema;

        Path path1 = new Path(segments1, -Math.PI / 6);
        path1.transverseVelocity = 1.5;

        Path path2 = new Path(segments2, -Math.PI / 6);
        path2.transverseVelocity = 1.5;


        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path1), alignSchema, drivetrain), 
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder, range1),
                new PivotRangeCommand(pivot, rangeFinder, range1)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0),     
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path2), alignSchema, drivetrain),
                new SequentialCommandGroup(
                    collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter),
                    new ParallelCommandGroup(
                        new RunShooter(shooter, range1),
                        new PivotRangeCommand(pivot, range1)
                    )
                )      
            ), 
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder, 4.3),
                new PivotRangeCommand(pivot, rangeFinder, 4.3)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0)
        );
    }
}
