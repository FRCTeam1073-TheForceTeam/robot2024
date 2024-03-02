package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class BlueCloseMidline3Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> points = new ArrayList<Pose2d>();
        points.add(new Pose2d(1.19, -0.97, new Rotation2d(0.44)));
        points.add(new Pose2d(7.37, 0.1, new Rotation2d(0.0)));
        points.add(new Pose2d(5.0, -0.14, new Rotation2d(0.33)));
        points.add(new Pose2d(7.4, -1.6, new Rotation2d(0.0)));
        points.add(new Pose2d(5.0, -0.14, new Rotation2d(0.33)));

        return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, points, 3.0, 3.0, 8.0), m_drivetrain);
    }
    
}
