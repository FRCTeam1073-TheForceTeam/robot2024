package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSpeakerAutoSchema;
import frc.robot.commands.CollectFeedCommand;
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
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Headlight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RangeFinder;
import frc.robot.subsystems.Shooter;

public class CenterL3
{
    public static Command create(Drivetrain drivetrain, Headlight headlight, Shooter shooter, Pivot pivot, Feeder feeder, CollectFeedCommand collectCommand, 
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

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(0.6308, 0.2343 * allianceSign);
        Path.Point wingNote7 = new Path.Point(1.4938, -0.202768 * allianceSign);
        //Path.Point btwPoint = new Path.Point(0.8090, 0.3534 * allianceSign);
        Path.Point wingNote6 = new Path.Point(1.331, 1.0337 * allianceSign);
        Path.Point afterWingNote6 = new Path.Point(0.67937, -0.435164 * allianceSign);

        double range1 = 1.41;
        double range2 = 1.61;
        double range3 = 1.8;
        //Pose2d poseShootPoint = new Pose2d();

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(start, pathShootPoint, 0 * allianceSign, 3.0)); //TODO: ask about the orientations
        //segments1.add(new Segment(shootPoint, wingNote7, 0, 2.5));
        segments1.get(0).entryActivateValue = true;
        segments1.get(0).entryActivate = alignSchema;
        segments1.get(0).exitActivateValue = false;
        segments1.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(pathShootPoint, wingNote6, 0.0 * allianceSign, 3.0));
        //segments2.add(new Segment(wingNote7, pathShootPoint, -0.768, 3.0));

        segments2.get(0).entryActivateValue = true;
        segments2.get(0).entryActivate = alignSchema;
        segments2.get(0).exitActivateValue = false;
        segments2.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments3 = new ArrayList<Segment>();
        segments3.add(new Segment(wingNote6, pathShootPoint, 0 * allianceSign, 3.0)); //TODO: ask about the orientations
        //segments1.add(new Segment(shootPoint, wingNote7, 0, 2.5));
        segments3.get(0).entryActivateValue = true;
        segments3.get(0).entryActivate = alignSchema;
        segments3.get(0).exitActivateValue = false;
        segments3.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments4 = new ArrayList<Segment>();
        segments4.add(new Segment(pathShootPoint, wingNote7, 0.0 * allianceSign, 3.0));

        segments4.get(0).entryActivateValue = true;
        segments4.get(0).entryActivate = alignSchema;
        segments4.get(0).exitActivateValue = false;
        segments4.get(0).exitActivate = alignSchema;

        ArrayList<Segment> segments5 = new ArrayList<Segment>();
        segments5.add(new Segment(wingNote7, afterWingNote6, 0.0 * allianceSign, 3.0));

        segments5.get(0).entryActivateValue = true;
        segments5.get(0).entryActivate = alignSchema;
        segments5.get(0).exitActivateValue = false;
        segments5.get(0).exitActivate = alignSchema;


        Path path1 = new Path(segments1, -0.11083 * allianceSign);
        path1.transverseVelocity = 5.0;

        Path path2 = new Path(segments2, 0.2538 * allianceSign); 
        path2.transverseVelocity = 5.0;
        
        Path path3 = new Path(segments3, -0.0524 * allianceSign);
        path3.transverseVelocity = 5.0;

        Path path4 = new Path(segments4, -0.680608 * allianceSign);
        path4.transverseVelocity = 5.0;

        Path path5 = new Path(segments5, -0.14333 * allianceSign);
        path5.transverseVelocity = 5.0;

        return new SequentialCommandGroup(
            //first path - back up and shoot preload
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path1), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                new RunShooter(shooter, range1),
                new PivotRangeCommand(pivot, range1)
            ),
            //adjust shooter speed/pivot based on range finder for shot 1
            new ParallelCommandGroup(
                new RunShooter(shooter, rangeFinder, range1),
                new PivotRangeCommand(pivot, rangeFinder, range1)
            ),
            // shot #1
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0), 

        //second path - back up, collect note, shoot from that point
            new ParallelCommandGroup(
                new RunShooter(shooter, range2),
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path2), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                collectCommand.runCollectCommand(drivetrain, collector, collectorArm)
            ),
            // new ParallelCommandGroup(
            //     collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter), 
            //     SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path3), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain)
            // ),
            collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter), 
            
            new ParallelCommandGroup(  
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path3), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),  
                new RunShooter(shooter, rangeFinder, range1),
                new PivotRangeCommand(pivot, rangeFinder, range1)
            ),
            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),
            new NWSetPivot(pivot, 0.0),
        //third path
            // 3rd shot
            new ParallelCommandGroup(
                new RunShooter(shooter, range3),
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path4), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                collectCommand.runCollectCommand(drivetrain, collector, collectorArm)
            ),
            collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter), 

            // new ParallelCommandGroup(
            //     collectCommand.runCollectFeedCommand(drivetrain, collector, collectorArm, pivot, feeder, shooter), 
            //     SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path5), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain)
            // ),

             new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path5), new AlignSpeakerAutoSchema(tagFinder, headlight), drivetrain),
                new RunShooter(shooter, rangeFinder, range3),
                new PivotRangeCommand(pivot, rangeFinder, range3)
            ),

            new ParallelCommandGroup(
                new RunFeeder(feeder, 30),
                new NWStopShooter(shooter)
            ),

            new NWSetPivot(pivot, 0.0)
        );
    }
}
