package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class RedCloseMidline2Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        ArrayList<Pose2d> pointList = new ArrayList<Pose2d>();
        pointList.add(new Pose2d(0.65, 1.1, new Rotation2d(-Math.PI / 6)));
        pointList.add(new Pose2d(6.5, -0.1, new Rotation2d(0)));
        pointList.add(new Pose2d(3.9, 0.0, new Rotation2d(-0.227)));
        return SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, pointList, 2.0, 2.0, 15.0), m_drivetrain);
    }    
}
