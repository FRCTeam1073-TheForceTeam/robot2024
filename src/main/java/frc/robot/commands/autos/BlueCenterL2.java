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
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Headlight;

public class BlueCenterL2 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, 
        CollectFeedCommand collectCommand, Collector collector, CollectorArm collectorArm, AprilTagFinder tagFinder, RangeFinder rangeFinder)
    {
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder, headlight);

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(0.9208, 0.3447);
        Path.Point wingNote7 = new Path.Point(1.5, 0.313);

        double range1 = 2.0;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(start, pathShootPoint, 0, 3.0)); //TODO: ask about the orientations
        //segments1.add(new Segment(shootPoint, wingNote7, 0, 2.5));
        segments1.get(0).entryActivateValue = true;
        segments1.get(0).entryActivate = alignSchema;
        segments1.get(0).exitActivateValue = false;
        segments1.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(pathShootPoint, wingNote7, 0.0, 3.0));
        //segments2.add(new Segment(wingNote7, pathShootPoint, -0.768, 3.0));

        segments2.get(0).entryActivateValue = true;
        segments2.get(0).entryActivate = alignSchema;
        segments2.get(0).exitActivateValue = false;
        segments2.get(0).exitActivate = alignSchema;

        Path path1 = new Path(segments1, 0.0524);
        path1.transverseVelocity = 1.5;

        Path path2 = new Path(segments2, 0.1361);
        path2.transverseVelocity = 1.5;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path1), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder, range1),
                new PivotRangeCommand(pivot, rangeFinder, range1)
            ),
            new RunFeeder(feeder, 30),
    
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path2), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            ),
                new NWSetPivot(pivot, 0.0)
        );
    }    
}
