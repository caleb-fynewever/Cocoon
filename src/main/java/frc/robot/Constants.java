package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.TagTracker.TagTrackerConstants;

public class Constants {
    public static class DrivetrainConstants {
        // Left-to-right distance between drivetrain wheels
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24);
        // Front-to-back distance between drivetrain wheels
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19);
        // translation from center of drivetrain to center of robot
        public static final Translation2d DRIVETRAIN_TO_ROBOT_CENTER_METERS = new Translation2d(
                Units.inchesToMeters(2.25),
                0);

        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.2)
                .withKS(0).withKV(1.5).withKA(0);
    
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(3).withKI(0).withKD(0)
                .withKS(0).withKV(0).withKA(0);

        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip
        private static final double kSlipCurrentA = 150.0;
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output
        public static final double kSpeedAt12VoltsMps = 4.70;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
        private static final double kCoupleRatio = 3.5;

        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        private static final double WHEEL_RADIUS_INCHES = 3.95;

        private static final boolean STEER_REVERSED = true;
        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = true;

        private static final String CAN_BUS_NAME = "drivetrain";

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;
        
        private static final TalonFXConfiguration initialDriveConfigs = new TalonFXConfiguration();
        static {
            initialDriveConfigs.CurrentLimits.SupplyCurrentLimit = 80;
            initialDriveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            initialDriveConfigs.Audio.BeepOnBoot = false;
            initialDriveConfigs.Audio.BeepOnConfig = false;
            initialDriveConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
            initialDriveConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.01;
            initialDriveConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
        }
        private static final TalonFXConfiguration initialSteerConfigs = new TalonFXConfiguration();
        static {
            initialSteerConfigs.Audio.BeepOnBoot = false;
            initialSteerConfigs.Audio.BeepOnConfig = false;
            initialSteerConfigs.CurrentLimits.StatorCurrentLimit = 50.0;
            initialSteerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(CAN_BUS_NAME)
                .withPigeon2Id(Ports.PIDGEON_ID)
                .withPigeon2Configs(pigeonConfigs);

        public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_REDUCTION)
                .withSteerMotorGearRatio(STEER_REDUCTION)
                .withWheelRadius(WHEEL_RADIUS_INCHES)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(STEER_REVERSED)
                .withDriveMotorInitialConfigs(initialDriveConfigs)
                .withSteerMotorInitialConfigs(initialSteerConfigs)
                .withCANcoderInitialConfigs(cancoderInitialConfigs);

        public static final class FrontLeftModule {
            private static final double FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(0.0);

            private static final double FRONT_LEFT_X_POS_INCHES = 10.5;
            private static final double FRONT_LEFT_Y_POS_INCHES = 10.5;

            public static SwerveModuleConstants getConstants() {
                return ConstantCreator.createModuleConstants(
                    Ports.FL_STEER,
                    Ports.FL_DRIVE,
                    Ports.FL_ENCODER,
                    FRONT_LEFT_ENCODER_OFFSET,
                    Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES),
                    Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
                    INVERT_LEFT_SIDE);
            }
        }

        public static final class FrontRightModule {
            private static final double FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(0.0);

            private static final double FRONT_RIGHT_X_POS_INCHES = 10.5;
            private static final double FRONT_RIGHT_Y_POS_INCHES = 10.5;

            public static SwerveModuleConstants getConstants() {
                return ConstantCreator.createModuleConstants(
                    Ports.FR_STEER,
                    Ports.FR_DRIVE,
                    Ports.FR_ENCODER,
                    FRONT_RIGHT_ENCODER_OFFSET,
                    Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES),
                    Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
                    INVERT_RIGHT_SIDE);
            }
        }

        public static final class BackLeftModule {
            private static final double BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(0.0);

            private static final double BACK_LEFT_X_POS_INCHES = 10.5;
            private static final double BACK_LEFT_Y_POS_INCHES = 10.5;

            public static SwerveModuleConstants getConstants() {
                return ConstantCreator.createModuleConstants(
                    Ports.BL_STEER,
                    Ports.BL_DRIVE,
                    Ports.BL_ENCODER,
                    BACK_LEFT_ENCODER_OFFSET,
                    Units.inchesToMeters(BACK_LEFT_X_POS_INCHES),
                    Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES),
                    INVERT_LEFT_SIDE);
            }
        }

        public static final class BackRightModule {
            private static final double BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(0.0);

            private static final double BACK_RIGHT_X_POS_INCHES = 10.5;
            private static final double BACK_RIGHT_Y_POS_INCHES = 10.5;

            public static SwerveModuleConstants getConstants() {
                return ConstantCreator.createModuleConstants(
                    Ports.BR_STEER,
                    Ports.BR_DRIVE,
                    Ports.BR_ENCODER,
                    BACK_RIGHT_ENCODER_OFFSET,
                    Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES),
                    Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
                    INVERT_RIGHT_SIDE);
            }
        }

        public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);

        public static final double COLLISION_THRESHOLD_DELTA_G = 1f;

        /*
         * Lower means less priority for driving
         * Driver: 0
         * Auto: 1
         * Aim: 2
         * Snap: 3
         */

        public static final int DRIVER_PRIORITY = 0;
        public static final int AUTO_PRIORITY = 1;
        public static final int SNAP_PRIORITY = 2;
        public static final int AIM_PRIORITY = 3;
    }
    
    public static class VisionConstants {
        public static final double XY_STDDEV = 0.7;
        public static final double HEADING_STDDEV = 99;
        public static final Matrix<N3, N1> VISION_STDDEV = VecBuilder.fill(XY_STDDEV, XY_STDDEV, HEADING_STDDEV);

        public static final double MAX_POSE_AMBIGUITY = 0.15;
        public static final double FIELD_BORDER_MARGIN = 0.5;
        public static final double MAX_VISION_CORRECTION = 2;

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo
                .loadAprilTagLayoutField();

        /*
         * Camera Order:
         * 0 1
         * 2 3
         */

        /* Front Left Camera */
        public static final class Camera0Constants {
            public static final String CAMERA_NAME = "KrawlerCam_FL_000";

            public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            public static final double X_OFFSET_M = 0.29;
            public static final double Y_OFFSET_M = 0.26;
            public static final double Z_OFFSET_M = 0.25;

            public static final double THETA_X_OFFSET_DEGREES = 0; // roll
            public static final double THETA_Y_OFFSET_DEGREES = -45; // pitch
            public static final double THETA_Z_OFFSET_DEGREES = 0; // yaw

            public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
                    new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
                    new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Y_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Z_OFFSET_DEGREES)));

            public static TagTrackerConstants TagTrackerConstants() {
                return new TagTrackerConstants(
                        CAMERA_NAME,
                        ROBOT_TO_CAMERA_METERS,
                        VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                        STRATEGY);
            }
        }

        /* Front Right Camera */
        public static final class Camera1Constants {
            public static final String CAMERA_NAME = "KrawlerCam_FR_001";

            public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            public static final double X_OFFSET_M = 0.29;
            public static final double Y_OFFSET_M = -0.26;
            public static final double Z_OFFSET_M = 0.25;

            public static final double THETA_X_OFFSET_DEGREES = 0.0; // roll
            public static final double THETA_Y_OFFSET_DEGREES = -30; // pitch
            public static final double THETA_Z_OFFSET_DEGREES = 0.0; // yaw

            public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
                    new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
                    new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Y_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Z_OFFSET_DEGREES)));

            public static TagTrackerConstants TagTrackerConstants() {
                return new TagTrackerConstants(
                        CAMERA_NAME,
                        ROBOT_TO_CAMERA_METERS,
                        VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                        STRATEGY);
            }
        }

        /* Back Left Camera */
        public static final class Camera2Constants {
            public static final String CAMERA_NAME = "KrawlerCam_BL_002";

            public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            public static final double X_OFFSET_M = -0.350;
            public static final double Y_OFFSET_M = 0.202;
            public static final double Z_OFFSET_M = 0.318;

            public static final double THETA_X_OFFSET_DEGREES = 0; // roll
            public static final double THETA_Y_OFFSET_DEGREES = -17; // pitch
            public static final double THETA_Z_OFFSET_DEGREES = 178; // yaw

            public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
                    new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
                    new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Y_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Z_OFFSET_DEGREES)));

            public static TagTrackerConstants TagTrackerConstants() {
                return new TagTrackerConstants(
                        CAMERA_NAME,
                        ROBOT_TO_CAMERA_METERS,
                        VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                        STRATEGY);
            }
        }

        /* Back Right Camera */
        public static final class Camera3Constants {
            public static final String CAMERA_NAME = "KrawlerCam_BR_003";

            public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            public static final double X_OFFSET_M = -0.350;
            public static final double Y_OFFSET_M = -0.210;
            public static final double Z_OFFSET_M = 0.318;

            public static final double THETA_X_OFFSET_DEGREES = 0; // roll
            public static final double THETA_Y_OFFSET_DEGREES = -15; // pitch
            public static final double THETA_Z_OFFSET_DEGREES = -178; // yaw

            public static final Transform3d ROBOT_TO_CAMERA_METERS = new Transform3d(
                    new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
                    new Rotation3d(Units.degreesToRadians(THETA_X_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Y_OFFSET_DEGREES),
                            Units.degreesToRadians(THETA_Z_OFFSET_DEGREES)));

            public static TagTrackerConstants TagTrackerConstants() {
                return new TagTrackerConstants(
                        CAMERA_NAME,
                        ROBOT_TO_CAMERA_METERS,
                        VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                        STRATEGY);
            }
        }
    }

    
    public static class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
        public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);
    }
}
