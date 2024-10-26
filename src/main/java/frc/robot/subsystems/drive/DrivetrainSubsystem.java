package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionUpdate;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private RobotState robotState;

    public DrivetrainSubsystem(RobotState robotState, SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, modules);
        configurePathPlanner();

        this.robotState = robotState;

        if(Robot.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;

        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                PathPlannerConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                //TODO: uhhh this red or blue might be broken
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void addVisionMeasurement(VisionUpdate visionUpdate) {
        if (visionUpdate.getVisionMeasurementStdDevs() == null) {
            this.addVisionMeasurement(
                    visionUpdate.estimatedPose.toPose2d(), visionUpdate.timestampSeconds);
        } else {
            this.addVisionMeasurement(
                    visionUpdate.estimatedPose.toPose2d(),
                    visionUpdate.timestampSeconds,
                    visionUpdate.getVisionMeasurementStdDevs());
        }
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        
        robotState.addDrivetrainState(super.getState());
    }

    public static double getMaxVelocityMetersPerSecond() {
        return TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    }

    public static double getMaxAngularVelocityRadiansPerSecond() {
        /*
         * Find the theoretical maximum angular velocity of the robot in radians per
         * second
         * (a measure of how fast the robot can rotate in place).
         */

        return getMaxVelocityMetersPerSecond() / Math.hypot(
                DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });

        simNotifier.startPeriodic(kSimLoopPeriod);
    }
}