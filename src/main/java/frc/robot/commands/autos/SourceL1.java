package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.NWSetPivot;
import frc.robot.commands.NWStopShooter;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class SourceL1 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, 
        AprilTagFinder tagFinder, RangeFinder rangeFinder, boolean isRed)
    {
        int allianceSign = 0;
        if (isRed)
        {
            allianceSign = 1;
        }
        else
        {
            allianceSign = -1;
        }
        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder);

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.165, -0.848 * allianceSign);

        double range1 = 3.9;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, Math.PI / 6 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;

        Path path = new Path(segments, Math.PI / 6 * allianceSign);
        path.transverseVelocity = 1.5;

        return new ParallelCommandGroup(
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), new AlignSpeakerAutoSchema(tagFinder), drivetrain), 
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            new SequentialCommandGroup(
                new RunShooter(shooter, rangeFinder),
                new PivotRangeCommand(pivot, rangeFinder),
                new ParallelCommandGroup(
                    new RunFeeder(feeder, 30),
                    new NWStopShooter(shooter)
                ),
                new NWSetPivot(pivot, 0.0)
            )
        );
    }

}
