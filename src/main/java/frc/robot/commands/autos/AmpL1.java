package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.NWSetPivot;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Headlight;

public class AmpL1 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, 
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

        AlignSpeakerAutoSchema alignSchema = new AlignSpeakerAutoSchema(tagFinder, headlight);

        Path.Point startPoint = new Path.Point(0.0, 0.0);
        Path.Point shootPoint = new Path.Point(1.757, 0.109 * allianceSign);

        double range1 = 3.15;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(startPoint, shootPoint, -0.724 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;

        Path path = new Path(segments, -0.724 * allianceSign);

        return new SequentialCommandGroup(
            new WaitCommand(8.0),
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), alignSchema, drivetrain),
                new ParallelCommandGroup(
                    //warm up shooter
                    new RunShooter(shooter, range1),
                    //warm up pivot
                    new PivotRangeCommand(pivot, range1)
                )
            ),
            new ParallelCommandGroup(
                //shoot shot
                new RunShooter(shooter, rangeFinder),
                //find pivot
                new PivotRangeCommand(pivot, rangeFinder)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new StopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0)
        );
    }    
}
