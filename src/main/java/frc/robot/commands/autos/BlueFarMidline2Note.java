package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class BlueFarMidline2Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> points = new ArrayList<Pose2d>();
        points.add(new Pose2d(1.25, 1.1, new Rotation2d(-Math.PI / 6)));
        points.add(new Pose2d(6.7, -1.2, new Rotation2d(0.0)));
        points.add(new Pose2d(3.48, -0.47, new Rotation2d(0.43)));

        return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, points, 3.0, 3.0, 8.0), m_drivetrain);
    }
    
}
