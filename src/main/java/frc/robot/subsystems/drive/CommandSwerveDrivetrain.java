package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

/**
 * This is a simple container for holding CTRE drive creation constants. 
 * This is done so that we can copy and paste in new TunerConstants.java if we need to change configuration and want to use the wizard again.
 */
public class CommandSwerveDrivetrain {

    SwerveDrivetrainConstants driveTrainConstants;
    SwerveModuleConstants[] moduleConstants;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this.driveTrainConstants = driveTrainConstants;
        this.moduleConstants = modules;
    }

    public SwerveDrivetrainConstants getDriveTrainConstants() {
        return driveTrainConstants;
    }

    public SwerveModuleConstants[] getModuleConstants() {
        return moduleConstants;
    }
}
