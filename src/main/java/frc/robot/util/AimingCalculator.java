package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class AimingCalculator {
  public static final double ToFFactor = 0.2;
  public static final boolean veloAim = false;

  /**
   * Get speaker shot parames
   *
   * @param robotPose Pose of the robot
   * @param chassisSpeeds FIELD relative chassis speeds of the robot
   * @param target Target to aim to
   * @return Array with length of 2 including distance and goal angle of the robot
   */
  public static double[] aimToPoint(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, Translation2d target) {
    Translation2d robotToTarget = target.minus(robotPose.getTranslation());
    double yaw = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
    double distanceToTarget = robotToTarget.getNorm();

    if (veloAim) {
      double[] shootOnMoveParams =
          getAdjustedShootOnMoveParams(yaw, distanceToTarget, chassisSpeeds);
      yaw = shootOnMoveParams[0];
      distanceToTarget = shootOnMoveParams[1];
    }

    yaw += Math.PI;
    // yaw = MathUtil.inputModulus(yaw, 0, 360);

    Logger.recordOutput("AIMING DIST", distanceToTarget);
    Logger.recordOutput("AIMING ANGLE", Units.radiansToDegrees(yaw));

    return new double[] {distanceToTarget, yaw};
  }

  private static double[] getAdjustedShootOnMoveParams(
      double uncompensatedYaw, double uncompensatedDistance, ChassisSpeeds chassisSpeeds) {
    Translation2d polarVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(Rotation2d.fromDegrees(uncompensatedYaw));
    double theta = polarVelocity.getX(); // velocity towards the target
    double r = polarVelocity.getY(); // velocity perpendicular to the target

    double shotSpeed =
        uncompensatedDistance / ToFFactor
            - theta; // distance divided by time of flight minus the velocity toward the target
    shotSpeed = Math.max(0.0, shotSpeed); // makes the shot speed at lowest zero
    double yawAdjustment = Math.atan2(-r, shotSpeed);
    double distanceAdjusted = ToFFactor * Math.hypot(r, shotSpeed);
    return new double[] {yawAdjustment + uncompensatedYaw, distanceAdjusted};
  }
}
