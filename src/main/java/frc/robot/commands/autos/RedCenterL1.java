package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.StopShooter;
import frc.robot.commands.WaitForPoint;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class RedCenterL1 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, CollectFeedCommand collectCommand, 
    Collector collector, CollectorArm collectorArm, AprilTagFinder tagFinder, RangeFinder rangeFinder)
    {
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder);

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(1.31, 0.37);

        double range1 = 2.3;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, 0.13, 2.5)); 

        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;
        
        Path path = new Path(segments, 0.13);
        path.transverseVelocity = 1.5;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), new AlignSpeakerAutoSchema(tagFinder), drivetrain),
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder),
                new PivotRangeCommand(pivot, rangeFinder)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            ),
                new SetPivotCommand(pivot, 0.0)
        );
    }
}
