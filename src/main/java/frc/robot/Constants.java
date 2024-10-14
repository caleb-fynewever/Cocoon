package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

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
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
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
        
        public static final SwerveDrivetrainConstants TUNER_DRIVETRAIN_CONSTANTS = TunerConstants.DriveTrain.getDriveTrainConstants();
        public static final SwerveModuleConstants[] TUNER_MODULE_CONSTANTS = TunerConstants.DriveTrain.getModuleConstants();
        public static final double MAX_SPEED = 4.43676260556;

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
