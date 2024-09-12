package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.CollectFeedCommand;
import frc.robot.commands.CollectorIntakeCommand;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.NWSetPivot;
import frc.robot.commands.NWStopShooter;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.WaitForShot;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class SourceL1 
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, 
        Collector collector, CollectorArm collectorArm, AprilTagFinder tagFinder, RangeFinder rangeFinder, boolean isRed)
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
        Double degreesToRadians = Math.PI / 180;

        Path.Point start = new Path.Point(0.0, 0.0);
        //Path.Point pathShootPoint = new Path.Point(3.165, -0.848 * allianceSign);
        Path.Point pathShootPoint = new Path.Point(2.921, -0.128 * allianceSign);
        
        Path.Point secondRingOne = new Path.Point(5.326, 0.505 * allianceSign);
        Path.Point secondRingTwo = new Path.Point(7.396, -0.372 * allianceSign);

        double range1 = 5.6;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, Math.PI / 6 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;

        Path path = new Path(segments, 41.239 * degreesToRadians * allianceSign);
        path.transverseVelocity = 1.5;

        ArrayList<Segment> secondSegment = new ArrayList<Segment>();
        segments.add(new Segment(pathShootPoint, secondRingOne, Math.PI / 6 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;      

        Path secondPath = new Path(secondSegment, 2.123 * degreesToRadians * allianceSign);
        path.transverseVelocity = 1.5;

        ArrayList<Segment> thirdSegment = new ArrayList<Segment>();
        segments.add(new Segment(secondRingOne, secondRingTwo, Math.PI / 6 * allianceSign, 2.5));
        segments.get(0).entryActivateValue = true;
        segments.get(0).entryActivate = alignSchema;
        segments.get(0).exitActivateValue = false;
        segments.get(0).exitActivate = alignSchema;
        
        Path thirdPath = new Path(thirdSegment, 7.526 * degreesToRadians * allianceSign);
        path.transverseVelocity = 1.5;

        // return new ParallelCommandGroup(
        //     new WaitCommand(10.0),
        //     new ParallelCommandGroup(
        //         SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain), 
        //         new RunShooter(shooter, range1),
        //         new PivotRangeCommand(pivot, range1)
        //     ),
        //     new SequentialCommandGroup(
        //         new RunShooter(shooter, rangeFinder),
        //         new PivotRangeCommand(pivot, rangeFinder),
        //         new ParallelCommandGroup(
        //             new RunFeeder(feeder, 30),
        //             new NWStopShooter(shooter)
        //         ),
        //         new NWSetPivot(pivot, 0.0)
        //     )
        // );

        return new SequentialCommandGroup(
            new WaitCommand(1.0),
            //ramps up and pivots the shoot as the drive base moves to the shooting position
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), alignSchema, drivetrain), 
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            //ramps up the shooter
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder, range1), 
                new PivotRangeCommand(pivot, rangeFinder, range1)
            ),
            //shoot the note and then stop the shooter
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0),
            
            //take new waypoints to get to note
            new SequentialCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, secondPath), alignSchema, drivetrain),
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, thirdPath), alignSchema, drivetrain),
                new CollectorIntakeCommand(collector, collectorArm, drivetrain)
            )
        );
    }

}
