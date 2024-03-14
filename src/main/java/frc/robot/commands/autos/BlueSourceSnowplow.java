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

public class BlueSourceSnowplow 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder)
    {
        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.5, 0.0);
        Path.Point midlineLeft = new Path.Point(8.26, -1.48); 
        Path.Point midlineRight = new Path.Point(8.26, 3.49); 

        Pose2d poseShootPoint = new Pose2d(3.5, 0.0, new Rotation2d(-0.83));
        double range1 = 4.1;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        segments1.add(new Segment(start, pathShootPoint, -0.83, 2.5));

        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        segments2.add(new Segment(pathShootPoint, midlineLeft, Math.PI / 4, 2.5));
        segments2.add(new Segment(midlineLeft, midlineRight, Math.PI / 4, 2.5));

        Path path1 = new Path(segments1, -0.83);
        path1.transverseVelocity = 1.5;

        Path path2 = new Path(segments1, Math.PI / 4);

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path1), drivetrain),
                new ParallelCommandGroup(
                    new RunShooter(shooter, range1),
                    new PivotRangeCommand(pivot, range1)
                )
            ),
            new WaitForPoint(drivetrain, poseShootPoint, 0.3, 0.25),
            new SequentialCommandGroup(                
                new ParallelCommandGroup(
                    new RunFeeder(feeder, 30),
                    new StopShooter(shooter)
                ),
                new SetPivotCommand(pivot, 0.0)
            ), 
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path2), drivetrain)
        );
    }    
}
