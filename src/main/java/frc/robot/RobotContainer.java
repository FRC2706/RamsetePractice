// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.commands.ArcadeDriveWithJoystick;
import frc.robot.commands.RamseteControllerLogging;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Joystick driverStick;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        driverStick = new Joystick(0);

        // Set the default drive command to split-stick arcade drive
        Command driveCommand = new ArcadeDriveWithJoystick(driverStick, Config.LEFT_CONTROL_STICK_Y,
                Config.INVERT_FIRST_AXIS, Config.RIGHT_CONTROL_STICK_X, Config.INVERT_SECOND_AXIS, true);
        DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {


        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Config.kMaxSpeedMetersPerSecond,
                Config.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Config.kDriveKinematics);
                        

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(),
                new Pose2d(2, -1, new Rotation2d())),
                // Pass config
                config); 

        RamseteCommand ramsete = new RamseteCommand(
            exampleTrajectory, 
            DriveSubsystem.getInstance()::getPose, 
            new RamseteControllerLogging(Config.kRamseteB, Config.kRamseteZeta),
            Config.kDriveKinematics,
            DriveSubsystem.getInstance()::tankDriveVelocity, 
            DriveSubsystem.getInstance());

        // Reset odometry to the starting pose of the trajectory.
        DriveSubsystem.getInstance().resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramsete.andThen(() -> DriveSubsystem.getInstance().tankDriveVelocity(0, 0));
    }

}
