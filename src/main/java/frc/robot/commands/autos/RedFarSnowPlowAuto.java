package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.StartRecordingAutonomous;
import frc.robot.subsystems.Drivetrain;

public class RedFarSnowPlowAuto 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> pointList = new ArrayList<Pose2d>();
    pointList.add(new Pose2d(7.2, 0, new Rotation2d(0)));
    pointList.add(new Pose2d(7.2, -5.7, new Rotation2d(0)));
    return new SequentialCommandGroup(SchemaDriveAuto.create(
        new DriveThroughTrajectorySchema(m_drivetrain, pointList, 6.0, 5.0, 7.0), 
          m_drivetrain));
    }
}
