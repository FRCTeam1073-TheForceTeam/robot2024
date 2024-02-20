package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drivetrain;

public class PointAtTargetSchema extends MotionSchema
{
    double maxAngularVelocity;
    Pose2d targetPose;


    public PointAtTargetSchema(double maxAngularVelocity,  Pose2d targetPose)
    {
        this.maxAngularVelocity = maxAngularVelocity;
        this.targetPose = targetPose;
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        Pose2d currentPose = drivetrain.getOdometry();
        double xDiff = currentPose.getX() - targetPose.getX();
        double yDiff = currentPose.getY() - targetPose.getY();
        double desiredAngle = Math.atan(yDiff / xDiff);
        double angularVelocity = currentPose.getRotation().getRadians() - desiredAngle;

        setRotate(angularVelocity, 1.0);
    }
}
