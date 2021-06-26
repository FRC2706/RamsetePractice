// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class DriveSubsystem extends SubsystemBase {
    // Singleton instance of this class
    private static DriveSubsystem instance;

    // 4 Talons on deep space chassis at Erik's house
    WPI_TalonSRX leftLeader, rightLeader, leftFollower, rightFollower;

    // Pigeon
    PigeonIMU pigeon; 


    // DifferentialDrive for acrade drive
    DifferentialDrive m_differentialDrive;

    public static DriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }
    

    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {
        // Talons
        leftLeader = new WPI_TalonSRX(Config.LEFT_LEADER_CANID);
        rightLeader = new WPI_TalonSRX(Config.RIGHT_LEADER_CANID);
        leftFollower = new WPI_TalonSRX(Config.LEFT_FOLLOWER_CANID);
        rightFollower = new WPI_TalonSRX(Config.RIGHT_FOLLOWER_CANID);

        // DifferentialDrive for acrade drive
        m_differentialDrive = new DifferentialDrive(leftLeader, rightLeader); // <- left & right leader talons

        // Pigeon is on the leftFollower talon
        pigeon = new PigeonIMU(leftFollower);

        setTalonConfigurations();

    }

    private void setTalonConfigurations() {
        leftLeader.configFactoryDefault();
        rightLeader.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration(); 

        /** Put any settings in TalonSRXConfiguration here. */
        talonConfig.slot1.kF = Config.RAMSETE_KF;


        // Error Code check all TalonSRXConfiguration settings
        // Config all talon settings - automatically returns worst error
        ErrorCode leftMasterError = leftLeader.configAllSettings(talonConfig);
        ErrorCode rightMasterError = rightLeader.configAllSettings(talonConfig);

        // Config the encoder and check if it worked
        ErrorCode leftEncoderError = leftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        ErrorCode rightEncoderError = rightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); 

        /** Put any other settings here */
        // Set the motor inversions
        leftLeader.setInverted(Config.LEFT_LEADER_INVERTED);
        rightLeader.setInverted(Config.RIGHT_LEADER_INVERTED);

        leftFollower.setInverted(Config.LEFT_FOLLOWER_INVERTED);
        rightFollower.setInverted(Config.RIGHT_FOLLOWER_INVERTED); 

        // Set the encoder inversions
        leftLeader.setSensorPhase(Config.DRIVETRAIN_LEFT_SENSORPHASE);
        rightLeader.setSensorPhase(Config.DRIVETRAIN_RIGHT_SENSORPHASE);

        // Ramsete set to slot 1 so any future Driver controls can be slot 1
        leftLeader.selectProfileSlot(1, 0);
        rightLeader.selectProfileSlot(1, 0);

        // Set brake mode for testing so it immediatly stops moving if something goes wrong
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake); 
    }

    /**
     * Motor control method for arcade drive.
     * 
     * @param forwardVal The forward value
     * @param rotateVal The rotate value
     * @param squareInputs Square the inputs?
     */
    public final void arcadeDrive(double forwardVal, double rotateVal, boolean squareInputs) {
        m_differentialDrive.arcadeDrive(forwardVal, rotateVal, squareInputs);
    }

    private Rotation2d getCurrentAngle() {
        FusionStatus status = new FusionStatus();
        double angle = pigeon.getFusedHeading(status);

        ErrorCode error = status.lastError; // ErrorCode to check

        return Rotation2d.fromDegrees(angle);
    }


    /**
     * Sets the velocity of the left and right side of the drivetrain in meters per second.
     * Called by RamseteCommand
     */
    public void tankDriveVelocity(double leftVel, double rightVel) {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
