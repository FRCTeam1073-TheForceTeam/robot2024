package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.CollectorIntakeCommand;
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
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class AmpL2 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, AprilTagFinder tagFinder, 
        RangeFinder rangeFinder, Collector collector, CollectorArm collectorArm, CollectFeedCommand collectCommand, boolean isRed)
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
        Path.Point shootPoint = new Path.Point(1.7649722, 0.6477758 * allianceSign); //1.757 (x)  0.25(y)
        Path.Point collectShootPoint = new Path.Point(2.107, 0.44 * allianceSign); //(0.2 y)


        double range1 = 2.5;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(startPoint, shootPoint, -0.724 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(shootPoint, shootPoint, -0.2, 2.5));

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(shootPoint, collectShootPoint, -0.2 * allianceSign, 2.5));
        segments2.get(0).entryActivateValue  = true;
        segments2.get(0).entryActivate = alignSchema;
        segments2.get(0).exitActivateValue = false;
        segments2.get(0).exitActivate = alignSchema;

        Path path = new Path(segments, -0.724 * allianceSign);
        Path path1 = new Path(segments1, -0.5441317); //0.0  old(0.4)
        Path path2 = new Path(segments2, -0.5441317 * allianceSign); //-0/588

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
            new NWSetPivot(pivot, 0.0),
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path1), alignSchema, drivetrain),
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path2), alignSchema, drivetrain),
                new SequentialCommandGroup(
                    //collectCommand.runCollectCommand(drivetrain, collector, collectorArm),
                    //collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter, rangeFinder, tagFinder)
                    new CollectorIntakeCommand(collector, collectorArm, drivetrain), 
                    collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter, rangeFinder, tagFinder)
                )
            ),
            /*new ParallelCommandGroup(
                new PivotRangeCommand(pivot, rangeFinder),
                new RunShooter(shooter, rangeFinder)
            ),
            */
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder),
                new PivotRangeCommand(pivot, rangeFinder)
            )
        );
    }       
}
