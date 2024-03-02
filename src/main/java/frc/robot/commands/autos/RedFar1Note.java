package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class RedFar1Note 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        return SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(3.0, 0.0, new Rotation2d(Math.PI / 4)), 5.0, 5.0), m_drivetrain);
    }
}
