package frc.robot;

import javax.sound.sampled.Port;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.team2052.lib.requests.DriveRequest;
import com.team2052.lib.requests.TurnRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output
        public static final double kSpeedAt12VoltsMps = 4.70;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
        private static final double kCoupleRatio = 3.5;

        private static final double DRIVE_REDUCTION = 7.363636364;
        private static final double STEER_REDUCTION = 15.42857143;
        private static final double WHEEL_RADIUS_INCHES = 2.167; // Estimated at first, then fudge-factored to make odom match record

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

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(CAN_BUS_NAME)
                .withPigeon2Id(Ports.PIDGEON_ID)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
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
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withCANcoderInitialConfigs(cancoderInitialConfigs);
        // Front Right
        private static final int kFrontRightDriveMotorId = 7;
        private static final int kFrontRightSteerMotorId = 6;
        private static final int kFrontRightEncoderId = 3;
        private static final double kFrontRightEncoderOffset = -0.15234375;

        private static final double kFrontRightXPosInches = 10.5;
        private static final double kFrontRightYPosInches = -10.5;

        // Back Left
        private static final int kBackLeftDriveMotorId = 1;
        private static final int kBackLeftSteerMotorId = 0;
        private static final int kBackLeftEncoderId = 0;
        private static final double kBackLeftEncoderOffset = -0.4794921875;

        private static final double kBackLeftXPosInches = -10.5;
        private static final double kBackLeftYPosInches = 10.5;

        // Back Right
        private static final int kBackRightDriveMotorId = 3;
        private static final int kBackRightSteerMotorId = 2;
        private static final int kBackRightEncoderId = 1;
        private static final double kBackRightEncoderOffset = -0.84130859375;

        private static final double kBackRightXPosInches = -10.5;
        private static final double kBackRightYPosInches = -10.5;
        /*
         * FL: 157.8 BL:3.5 BR:340.0 FR:188.9
         */

        public static final class FrontLeftModule {
            private static final double FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(DRIVETRAIN_TRACKWIDTH_METERS);

            private static final double FRONT_LEFT_X_POS_INCHES = 10.5;
            private static final double FRONT_LEFT_Y_POS_INCHES = 10.5;

            private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                    Ports.FR_STEER,
                    Ports.FR_DRIVE,
                    Ports.FR_ENCODER,
                    FRONT_LEFT_ENCODER_OFFSET,
                    Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES),
                    Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
                    INVERT_LEFT_SIDE);

        }

        public static final class FrontRightModule {
            public static final double STEER_OFFSET_RADIANS = -Math.toRadians(188.9);
            private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                    Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches),
                    INVERT_RIGHT_SIDE);

        }

        public static final class BackLeftModule {
            public static final double STEER_OFFSET_RADIANS = -Math.toRadians(3.5);

            private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                    Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
                    INVERT_LEFT_SIDE);
        }

        public static final class BackRightModule {
            public static final double STEER_OFFSET_RADIANS = -Math.toRadians(340);

            private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                    Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
                    INVERT_RIGHT_SIDE);
        }

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, DRIVETRAIN_WHEELBASE_METERS / 2),
                // Front right
                new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, -DRIVETRAIN_WHEELBASE_METERS / 2),
                // Back left
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, DRIVETRAIN_WHEELBASE_METERS / 2),
                // Back right
                new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, -DRIVETRAIN_WHEELBASE_METERS / 2));

        // public static final SwerveDriveKinematics kinematics = new
        // SwerveDriveKinematics(
        // // Front left
        // new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, 0.298),
        // // Front right
        // new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2, -0.298),
        // // Back left
        // new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, 0.178),
        // // Back right
        // new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2, -0.178)
        // );

        public static final Matrix<N3, N1> ODOMETRY_STDDEV = VecBuilder.fill(0.1, 0.1, 0.1);

        public static final double COLLISION_THRESHOLD_DELTA_G = 1f;

        public static final double AIM_TOL_DEG = 3.0;

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

        public static final double SNAP_PID_kP = 2.5;
        public static final double SNAP_PID_kI = 0.0;
        public static final double SNAP_PID_kD = 0.1;
        public static final double SNAP_PID_kF = 0.0;

        public static final double ROTATIONAL_FEEDFORWARD_kS = 0;// 0.61;
        public static final double ROTATIONAL_FEEDFORWARD_kV = 0;// 12 * DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond();
        public static final double ROTATIONAL_FEEDFORAWRD_kA = 0.0;

        public static final DriveRequest NULL_DRIVE = new DriveRequest(Integer.MIN_VALUE,
                new Translation2d(0, 0));
        public static final TurnRequest NULL_TURN = new TurnRequest(Integer.MIN_VALUE, 0);
    }
}
