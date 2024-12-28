package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionUpdate;
import java.util.Optional;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /* Keep track if we've ever applied the driver perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();

  private RobotState robotState = RobotState.getInstance();

  private static DrivetrainSubsystem INSTANCE;

  public static DrivetrainSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DrivetrainSubsystem();
    }

    return INSTANCE;
  }

  private DrivetrainSubsystem() {
    super(
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS, DrivetrainConstants.TUNER_MODULE_CONSTANTS);
    configurePathPlanner();

    if (Robot.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;

    for (var moduleLocation : getModuleLocations()) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configure(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::resetPose, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds, feedforwards) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())),
        PathPlannerConstants.PATH_FOLLOWING_CONTROLLER,
        PathPlannerConstants.ROBOT_CONFIG,
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red)),
        this); // Subsystem for requirements
  }

  // public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
  //     return run(() -> this.setControl(requestSupplier.get()));
  // }

  public void stop() {
    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
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
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0));
                hasAppliedOperatorPerspective = true;
              });
    }

    robotState.addDrivetrainState(super.getState());
  }

  public static double getMaxVelocityMetersPerSecond() {
    return TunerConstants.kSpeedAt12Volts.in(
        MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  }

  public static double getMaxAngularVelocityRadiansPerSecond() {
    /*
     * Find the theoretical maximum angular velocity of the robot in radians per
     * second
     * (a measure of how fast the robot can rotate in place).
     */

    return getMaxVelocityMetersPerSecond()
        / Math.hypot(
            DrivetrainConstants.DRIVETRAIN_TRACKWIDTH.in(Meters) / 2.0,
            DrivetrainConstants.DRIVETRAIN_WHEELBASE.in(Meters) / 2.0);
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
