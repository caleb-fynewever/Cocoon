package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.ctre.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.subsystems.vision.TagTracker.TagTrackerConstants;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Constants {

  public static class DriverConstants {
    public static final boolean FORCE_GAMEPAD = false;
    public static final double JOYSTICK_DEADBAND = 0.005;
    public static final double GAMEPAD_DEADBAND = 0.025; // add deadband here if there is drift
  }

  public static class DrivetrainConstants {
    public static final CommandSwerveDrivetrain TUNER_DRIVETRAIN_CONSTANTS =
        TunerConstants.createDrivetrain();
    /*
     * If using the generator, the order in which modules are constructed is
     * Front Left, Front Right, Back Left, Back Right. This means if you need
     * the Back Left module, call {@code getModule(2);} to get the 3rd index
     * (0-indexed) module, corresponding to the Back Left module.
     */

    public static final LinearVelocity DRIVE_MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity DRIVE_MAX_ANGULAR_RATE =
        RadiansPerSecond.of(DRIVE_MAX_SPEED.in(MetersPerSecond) / 0.42);

    public static final Current DRIVE_CURRENT_LIMIT_AMPS = Amps.of(80.0);

    public static final Distance WHEEL_RADIUS =
        Meters.of(TUNER_DRIVETRAIN_CONSTANTS.getModuleConstants()[0].WheelRadius);
    // Left-to-right distance between drivetrain wheels
    public static final Distance DRIVETRAIN_TRACKWIDTH = Inches.of(23.5);
    // Front-to-back distance between drivetrain wheels
    public static final Distance DRIVETRAIN_WHEELBASE = Inches.of(23.5);

    public static final Mass DRIVETRAIN_MASS = Pounds.of(100); // TODO: weigh the robot

    public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final Angle HEADING_TOLERANCE = Degrees.of(3);
  }

  public static class VisionConstants {
    public static final double XY_STDDEV = 0.7;
    public static final double HEADING_STDDEV = 99;
    public static final Matrix<N3, N1> VISION_STDDEV =
        VecBuilder.fill(XY_STDDEV, XY_STDDEV, HEADING_STDDEV);

    public static final double MAX_POSE_AMBIGUITY = 0.15;
    public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);
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

      public static final Distance X_OFFSET = Meters.of(0.29);
      public static final Distance Y_OFFSET = Meters.of(0.26);
      public static final Distance Z_OFFSET = Meters.of(0.25);

      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(-45); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(0); // yaw

      public static final Transform3d ROBOT_TO_CAMERA_METERS =
          new Transform3d(
              new Translation3d(X_OFFSET, Y_OFFSET, Z_OFFSET),
              new Rotation3d(THETA_X_OFFSET, THETA_Y_OFFSET, THETA_Z_OFFSET));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA_METERS, VisionConstants.APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(651.223);
    public static final Distance FIELD_WIDTH = Inches.of(323.277);
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

    // Rough estimation (1/12) * mass * (length^2 + width^2)
    public static final MomentOfInertia ROBOT_MOI =
        KilogramSquareMeters.of(
            (1 / 12)
                * DrivetrainConstants.DRIVETRAIN_MASS.in(Kilograms)
                * (Math.pow(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH.in(Meters), 2)
                    + Math.pow(DrivetrainConstants.DRIVETRAIN_WHEELBASE.in(Meters), 2)));

    public static final ModuleConfig MODULE_CONFIG =
        new ModuleConfig(
            DrivetrainConstants.WHEEL_RADIUS,
            DrivetrainConstants.DRIVE_MAX_SPEED,
            1.1,
            DCMotor.getKrakenX60Foc(1),
            DrivetrainConstants.DRIVE_CURRENT_LIMIT_AMPS,
            1);

    public static final RobotConfig ROBOT_CONFIG =
        new RobotConfig(
            DrivetrainConstants.DRIVETRAIN_MASS,
            ROBOT_MOI,
            MODULE_CONFIG,
            DrivetrainConstants.DRIVETRAIN_TRACKWIDTH,
            DrivetrainConstants.DRIVETRAIN_WHEELBASE);

    public static final PPHolonomicDriveController PATH_FOLLOWING_CONTROLLER =
        new PPHolonomicDriveController(
            new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
            new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD));
  }
}
