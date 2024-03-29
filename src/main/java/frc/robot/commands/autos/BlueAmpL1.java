package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.NWSetPivot;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.StopShooter;
import frc.robot.commands.WaitForPoint;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class BlueAmpL1 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, AprilTagFinder tagFinder, RangeFinder rangeFinder)
    {
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder, headlight);

        Path.Point startPoint = new Path.Point(0.0, 0.0);
        Path.Point shootPoint = new Path.Point(1.757, -0.109);

        double range1 = 2.5;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(startPoint, shootPoint, 0.724, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;

        Path path = new Path(segments, 0.724);

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                //drives while alinging
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), alignSchema, drivetrain),
                new ParallelCommandGroup(
                    //warm up shooter, oproximit
                    new RunShooter(shooter, range1),
                    //warm up pivot oproximit
                    new PivotRangeCommand(pivot, range1)
                )
            ),
            new ParallelCommandGroup(
                //shoot shot
                new RunShooter(shooter, range1),
                //find pivot
                new PivotRangeCommand(pivot, range1)
            ),
            new ParallelCommandGroup(
                //run feeder
                new RunFeeder(feeder, 30),
                //stop shooter
                new StopShooter(shooter)
            ),
            //sets pivot to 0
            new NWSetPivot(pivot, 0.0)
        );
    }
}
