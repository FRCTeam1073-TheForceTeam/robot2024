package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePathSchema;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class SourceSnowplow 
{
    public static Command create(Drivetrain drivetrain, Shooter shooter, Pivot pivot, Feeder feeder, boolean isRed)
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

        Path.Point start = new Path.Point(0.0, 0.0);
        Path.Point pathShootPoint = new Path.Point(3.5, 0.0);
        Path.Point midlineLeft = new Path.Point(8.26, 1.48 * allianceSign); 
        Path.Point midlineRight = new Path.Point(8.26, -3.49 * allianceSign); 

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, pathShootPoint, 0.83 * allianceSign, 3.5));
        segments.add(new Segment(pathShootPoint, midlineLeft, Math.PI / 4 * allianceSign, 3.5));
        segments.add(new Segment(midlineLeft, midlineRight, Math.PI / 4 * allianceSign, 3.5));
        

        Path path = new Path(segments, Math.PI / 4 * allianceSign);
        path.transverseVelocity = 1.5;

        return new SequentialCommandGroup(
            SchemaDriveAuto.create(new DrivePathSchema(drivetrain, path), drivetrain)
        );
    }
}
