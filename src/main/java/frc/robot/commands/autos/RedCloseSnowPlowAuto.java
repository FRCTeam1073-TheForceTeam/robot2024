package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.StartRecordingAutonomous;
import frc.robot.subsystems.Drivetrain;

public class RedCloseSnowPlowAuto
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> pointList = new ArrayList<Pose2d>();
        pointList.add(new Pose2d(7, 0, new Rotation2d(0)));
        pointList.add(new Pose2d(7, 5.5, new Rotation2d(0)));
        return new SequentialCommandGroup(SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(7, -1.5, new Rotation2d(0)), 5, 1), m_drivetrain),
          SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(7.0, 5.5, new Rotation2d(0)), 5, 1), m_drivetrain));
    }
}
