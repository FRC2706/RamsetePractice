# RamsetePractice
Practicing/Teaching ramsete to prepare for The Summer of Swerge

* Create your own branch from the RamsetePractice repo. I made a base code to start from so it's easier for me to follow what you guys add.
* Follow the Ramsete tutorial from WPILib with a few changes from the tutorial.. https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/index.html
* Skip over the page about RobotCharacterization.
* Instead of using the long constructor of RamseteCommand, use the shorter one. This one means it will return a velocity for the left and right side of the drivetrain instead of just a voltage. Then run a closed loop Velocity PID on the talon in order drive that desired velocity. Create a method in the DriveSubsystem called tankDriveVelocities to pass to the RamseteCommand constructor, add m_differentialDrive.feed() to tankDriveVelocities too.
* This also means we won't need RobotCharacterization or ks/kv/ka. On Step 4 of the tutorial remove the autoVoltageConstraint.
* Methods not needed in DriveSubsystem (step 3) are getWheelSpeeds, getAverageEncoderDistance, getTurnRate, and setMaxOutput, you can skip over these. (These were needed for encoder, arcade drive and pigeon stuff which I've already set up in the repo)
* Instead of using RamseteController use RamseteControllerLogging which is already in the RamsetePractice repo. (it... logs... to networktables) 
* The code from RamsetePractice already has some Config constants which are set to work on the deep space chassis that I have at home. Depending on how things go hopefully I can review all your code and test it.
