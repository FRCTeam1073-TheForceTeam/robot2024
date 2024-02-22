package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class PointAtTargetSchema extends MotionSchema
{
    double maxAngularVelocity;
    Pose2d targetPose;
    boolean active;
    double distanceTolerance;
    double alpha;

    public PointAtTargetSchema(double maxAngularVelocity,  Pose2d targetPose, double alpha)
    {
        this.maxAngularVelocity = maxAngularVelocity;
        this.targetPose = targetPose;
        this.alpha = alpha;
        active = false;
        distanceTolerance = 0.2;
    }

    public void setActive(boolean active)
    {
        this.active = active;
    }

    public boolean getActive()
    {
        return active;
    }

    @Override
    public void update(Drivetrain drivetrain)
    {
        Pose2d currentPose = drivetrain.getOdometry();
        double xDiff = targetPose.getX() - currentPose.getX();
        double yDiff = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        double desiredAngle = Math.atan2(yDiff, xDiff);
        if (yDiff == 0)
        {
            desiredAngle = 0;
        }
        double angleDifference = desiredAngle - currentPose.getRotation().getRadians();
        if (angleDifference > Math.PI)
        {
            angleDifference = 2 * Math.PI - angleDifference;
        }
        else if (angleDifference < -Math.PI)
        {
            angleDifference += 2 * Math.PI;
        }
        double angularVelocity = angleDifference * alpha;
        SmartDashboard.putNumber("Angle Difference", angleDifference);
        SmartDashboard.putNumber("Target Schema Angular Velocity", angularVelocity);

        if (active && distance > distanceTolerance)
        {
            setRotate(angularVelocity, 1.0);
        }
        else 
        {
            setRotate(0, 0);
        }
    }
}
