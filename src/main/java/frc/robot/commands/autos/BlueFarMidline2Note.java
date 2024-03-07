package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CollectorIntakeCommand;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.Path;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.WaitForPoint;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class BlueFarMidline2Note 
{
    public static Command create(Drivetrain m_drivetrain, Feeder feeder, Shooter shooter, Pivot pivot, 
        Collector collector, CollectorArm collectorArm, HandoffCommand handoffCommand)
    {
        Path.Point point1 = new Path.Point(0.0, 0.0);
        Path.Point point2 = new Path.Point(1.25, 1.1);
        Path.Point point3 = new Path.Point(6.7, -1.2);
        Path.Point point4 = new Path.Point(3.48, -0.47);

        Pose2d shootPoint1 = new Pose2d(1.25, 1.1, new Rotation2d(-Math.PI / 6));
        //Pose2d collectPoint = new Pose2d(6.5, -1.2, new Rotation2d());
        Pose2d shootPoint2 = new Pose2d(3.48, -0.47, new Rotation2d(0.43));

        double range1 = 0.0; // TODO: actually set these
        double range2 = 0.0;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(point1, point2, -Math.PI / 6, 2.5));
        segments.add(new Segment(point2, point3, 0.0, 2.5));
        segments.add(new Segment(point3, point4, 0.43, 2.5));

        return new ParallelCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(m_drivetrain, new Path(segments, 0.43)), m_drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunShooter(shooter, range1),
                    new PivotRangeCommand(pivot, range1)),
                new WaitForPoint(m_drivetrain, shootPoint1, 0.1, 0.1),
                new RunFeeder(feeder, 30),
                new CollectorIntakeCommand(collector, collectorArm, m_drivetrain), 
                handoffCommand.runHandoffCommand(m_drivetrain, collector, collectorArm, pivot, feeder, shooter),
                new ParallelCommandGroup(
                    new RunShooter(shooter, range2),
                    new PivotRangeCommand(pivot, range2)),
                new WaitForPoint(m_drivetrain, shootPoint2, 0.1, 0.1),
                new RunFeeder(feeder, 30)));
    }
    
}
