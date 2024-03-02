package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveThroughTrajectorySchema;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.commands.StartRecordingAutonomous;
import frc.robot.subsystems.Drivetrain;

public class BlueClose4Note 
{
    public static Command create(Drivetrain m_drivetrain, StartRecordingAutonomous c_startRecordingAutonomous)
    {
        ArrayList<Pose2d> pointList1 = new ArrayList<Pose2d>();
    ArrayList<Pose2d> pointList2 = new ArrayList<Pose2d>();
    pointList1.add(new Pose2d(1.0, -1.0, new Rotation2d(Math.PI / 4)));
    pointList1.add(new Pose2d(2.1, -2.1, new Rotation2d(Math.PI / 16)));
    pointList2.add(new Pose2d(1.0, -2.5, new Rotation2d(0)));
    pointList2.add(new Pose2d(2.1, -2.7, new Rotation2d(-Math.PI / 6)));
    return new SequentialCommandGroup(SchemaDriveAuto.create(
        new DriveToPointSchema(m_drivetrain, new Pose2d(1.75, -1, new Rotation2d(Math.PI / 4)), 5.0, 5.0), 
        m_drivetrain),
      SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(2.2, -0.5, new Rotation2d(Math.PI / 4)), 5.0, 5.0), m_drivetrain),
      SchemaDriveAuto.create(new DriveThroughTrajectorySchema(m_drivetrain, pointList1, 5.0, 5.0, 7.0), m_drivetrain),
      c_startRecordingAutonomous);
    }    
}
