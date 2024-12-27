package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.subsystems.vision.TagTracker.TagTrackerConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Constants {

  public static class DriverConstants {
    public static final boolean FORCE_GAMEPAD = false;
    public static final double JOYSTICK_DEADBAND = 0.075;
    public static final double GAMEPAD_DEADBAND = 0.0; // add deadband here if there is drift
  }

  public static class DrivetrainConstants {
    /*
     * If using the generator, the order in which modules are constructed is
     * Front Left, Front Right, Back Left, Back Right. This means if you need
     * the Back Left module, call {@code getModule(2);} to get the 3rd index
     * (0-indexed) module, corresponding to the Back Left module.
     */
    public static final SwerveDrivetrainConstants TUNER_DRIVETRAIN_CONSTANTS =
        TunerConstants.DriveTrain.getDrivetrainConstants();
    public static final SwerveModuleConstants[] TUNER_MODULE_CONSTANTS =
        TunerConstants.DriveTrain.getModuleConstants();
    public static final double MAX_SPEED = TUNER_MODULE_CONSTANTS[0].SpeedAt12Volts;

    public static final double DRIVE_CURRENT_LIMIT_AMPS = 80;

    public static final double WHEEL_RADIUS = TUNER_MODULE_CONSTANTS[0].WheelRadius;
    // Left-to-right distance between drivetrain wheels
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.5);
    // Front-to-back distance between drivetrain wheels
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.5);

    public static final double DRIVETRAIN_WEIGHT_KG =
        Units.lbsToKilograms(100); // TODO: weigh the robot

    public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final double COLLISION_THRESHOLD_DELTA_G = 1f;

    public static final double HEADING_TOLERANCE = 3;
  }

  public static class VisionConstants {
    public static final double XY_STDDEV = 0.7;
    public static final double HEADING_STDDEV = 99;
    public static final Matrix<N3, N1> VISION_STDDEV =
        VecBuilder.fill(XY_STDDEV, XY_STDDEV, HEADING_STDDEV);

    public static final double MAX_POSE_AMBIGUITY = 0.15;
    public static final double FIELD_BORDER_MARGIN = 0.5;
    public static final double MAX_VISION_CORRECTION = 2;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

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

      public static final Transform3d ROBOT_TO_CAMERA_METERS =
          new Transform3d(
              new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
              new Rotation3d(
                  Units.degreesToRadians(THETA_X_OFFSET_DEGREES),
                  Units.degreesToRadians(THETA_Y_OFFSET_DEGREES),
                  Units.degreesToRadians(THETA_Z_OFFSET_DEGREES)));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA_METERS, VisionConstants.APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }
  }

  public static class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);
  }

  public static final class DashboardConstants {
    public static final String DRIVE_MODE_KEY = "Drive Mode";
    public static final String AUTO_COMPILED_KEY = "Auto Compiled";
    public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
  }

  public static final class PathPlannerConstants {
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    public static final ModuleConfig MODULE_CONFIG =
        new ModuleConfig(
            DrivetrainConstants.WHEEL_RADIUS,
            DrivetrainConstants.MAX_SPEED,
            1.1,
            DCMotor.getKrakenX60Foc(1),
            DrivetrainConstants.DRIVE_CURRENT_LIMIT_AMPS,
            1);

    public static final RobotConfig ROBOT_CONFIG =
        new RobotConfig(
            DrivetrainConstants.DRIVETRAIN_WEIGHT_KG,
            (1 / 12)
                * DrivetrainConstants.DRIVETRAIN_WEIGHT_KG
                * (Math.pow(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS, 2)
                    + Math.pow(
                        DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS,
                        2)), // rough estimation (1/12) * mass * (length^2 + width^2)
            MODULE_CONFIG,
            DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS,
            DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS);

    public static final PPHolonomicDriveController PATH_FOLLOWING_CONTROLLER =
        new PPHolonomicDriveController(
            new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
            new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD));
  }
}
