package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.PivotRangeCommand;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.SetPivotCommand;
import frc.robot.commands.StopShooter;
import frc.robot.commands.WaitForPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class BlueSourceL1 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.5, 0.0);

        Pose2d poseShootPoint = new Pose2d(3.5, 0.0, new Rotation2d(-0.768));
        double range1 = 0.0;

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, -0.768, 2.5));

        Path path = new Path(segments, -0.768);
        path.transverseVelocity = 1.5;

        return new ParallelCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), drivetrain),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new RunShooter(shooter, range1),
                    new PivotRangeCommand(pivot, range1)
                ),
                new WaitForPoint(drivetrain, poseShootPoint, 0.25, 0.15),
                new ParallelCommandGroup(
                    new RunFeeder(feeder, 30),
                    new StopShooter(shooter)
                ),
                new SetPivotCommand(pivot, 0.0)
            )
        );
    }
}
