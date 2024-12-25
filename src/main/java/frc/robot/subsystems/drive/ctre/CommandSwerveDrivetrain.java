package frc.robot.subsystems.drive.ctre;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * This is a simple container for holding CTRE drive creation constants. This is done so that we can
 * copy and paste in new TunerConstants.java if we need to change configuration and want to use the
 * wizard again.
 */
public class CommandSwerveDrivetrain {

  SwerveDrivetrainConstants drivetrainConstants;
  SwerveModuleConstants[] moduleConstants;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... modules) {
    this.drivetrainConstants = drivetrainConstants;
    this.moduleConstants = modules;
  }

  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return drivetrainConstants;
  }

  public SwerveModuleConstants[] getModuleConstants() {
    return moduleConstants;
  }
}
