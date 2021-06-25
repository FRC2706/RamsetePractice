package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class RamseteControllerLogging extends RamseteController {
    
    NetworkTableEntry xError, yError, rotError;

    public RamseteControllerLogging() {
        super();

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainData");
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError");

    }

    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Trajectory.State desiredState) {
        ChassisSpeeds speeds = super.calculate(currentPose, desiredState);

        Pose2d error = desiredState.poseMeters.relativeTo(currentPose);

        xError.setDouble(error.getX());
        yError.setDouble(error.getY());
        rotError.setDouble(error.getRotation().getDegrees());
        
        return speeds;
    }
}
