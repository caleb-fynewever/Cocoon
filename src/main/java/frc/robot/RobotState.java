package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.MathHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionUpdate;

public class RobotState {
    private SwerveDriveState drivetrainState = new SwerveDriveState();

    private static RobotState INSTANCE;
    public static RobotState getInstance() {
        if(INSTANCE == null) {
            INSTANCE = new RobotState();
        } 
        
        return INSTANCE;
    }

    private RobotState() {}

    public Pose2d getFieldToRobot() {
        if(drivetrainState.Pose != null) {
            return drivetrainState.Pose;
        }

        return MathHelpers.POSE_2D_ZERO;
    }

    public ChassisSpeeds getChassisSpeeds(boolean isFieldRelative) {
        ChassisSpeeds chassisSpeeds = drivetrainState.Speeds;
        if (isFieldRelative) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getFieldToRobot().getRotation());
        } else {
            return chassisSpeeds;
        }
    }

    public void addDrivetrainState(SwerveDriveState drivetrainState) {
        this.drivetrainState = drivetrainState;
    }

    public void addVisionUpdate(VisionUpdate visionUpdate) {
        DrivetrainSubsystem.getInstance().addVisionMeasurement(visionUpdate);
    }
    
    /**
     * Returns true if the robot is on red alliance.
     *
     * @return True if the robot is on red alliance.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

    public void output() {
        Logger.recordOutput("Swerve Module States", drivetrainState.ModuleStates);
        Logger.recordOutput("Swerve Module Goals", drivetrainState.ModuleTargets);
        Logger.recordOutput("Current Pose", drivetrainState.Pose);
    }
}
