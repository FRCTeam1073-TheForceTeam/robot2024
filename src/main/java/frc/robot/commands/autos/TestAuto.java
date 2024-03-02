package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class TestAuto 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> pointList = new ArrayList<Pose2d>();
        pointList.add(new Pose2d(2.0, 0.0, new Rotation2d(0)));
        pointList.add(new Pose2d(2.0, 1.0, new Rotation2d(0)));
        pointList.add(new Pose2d(4.0, 1.0, new Rotation2d(Math.PI)));
        return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, pointList, 2.0, 2.0, 5.0), m_drivetrain);
    }
}
