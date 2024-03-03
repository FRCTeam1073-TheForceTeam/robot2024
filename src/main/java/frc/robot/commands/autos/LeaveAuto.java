package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPointSchema;
import frc.robot.commands.SchemaDriveAuto;
import frc.robot.subsystems.Drivetrain;

public class LeaveAuto 
{
    public static Command create(Drivetrain m_drivetrain)
    {
        return new SequentialCommandGroup(SchemaDriveAuto.create(new DriveToPointSchema(m_drivetrain, new Pose2d(1.5, 0.0, new Rotation2d()), 1.5, 0), m_drivetrain));
    }
    
}
