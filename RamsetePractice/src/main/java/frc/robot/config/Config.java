// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Config {

    // Set for Deep Space chassis at Erik's house
    public static final int LEFT_LEADER_CANID = 1;
    public static final int RIGHT_LEADER_CANID = 2;
    public static final int LEFT_FOLLOWER_CANID = 3;
    public static final int RIGHT_FOLLOWER_CANID = 4;

    public static final int PIGEON_CANID = 3;

    public static final boolean LEFT_LEADER_INVERTED = false;
    public static final boolean RIGHT_LEADER_INVERTED = true;
    public static final boolean LEFT_FOLLOWER_INVERTED = false;
    public static final boolean RIGHT_FOLLOWER_INVERTED = true;
    public static final boolean DRIVETRAIN_LEFT_SENSORPHASE = true;
    public static final boolean DRIVETRAIN_RIGHT_SENSORPHASE = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    public static final double drivetrainWheelDiameter = 0.1524;
    public static final double ticksPerRevolution = 4096;

    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double RAMSETE_KF = 0.5; // Rough/quick tuning.
    public static final double RAMSETE_KP = 0.15;
    public static final double RAMSETE_KI = 0;
    public static final double RAMSETE_KD = 0;
    public static final double RAMSETE_ALLOWABLE_PID_ERROR = 0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    /** Setup for ArcadeDrive */
    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;
    
    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;
    
    public static boolean INVERT_FIRST_AXIS = false;
    public static boolean INVERT_SECOND_AXIS = false;

    public static Double JOYSTICK_AXIS_DEADBAND = 0.1;

    /**
     * Used for AracdeDrive code from 2020
     * 
     * @param value The raw axis value from the control stick
     * @return The filtered value defined by the acceptable dead band
     */
    public static double removeJoystickDeadband(double value) {
        if (value <= JOYSTICK_AXIS_DEADBAND && value >= 0) {
            return 0;
        } else if (value >= -JOYSTICK_AXIS_DEADBAND && value <= 0) {
            return 0;
        } else {
            return value;
        }
    }

}
