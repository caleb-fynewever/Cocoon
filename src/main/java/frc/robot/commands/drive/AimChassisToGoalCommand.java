// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;

public class AimChassisToGoalCommand extends DefaultDriveCommand {
    private final RobotState robotState;

    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DrivetrainSubsystem.getMaxVelocityMetersPerSecond() * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    /**
     * @param xSupplier        supplier for forward velocity.
     * @param ySupplier        supplier for sideways velocity.
     * @param rotationSupplier supplier for angular velocity.
     */
    public AimChassisToGoalCommand (
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldCentricSupplier,
            DrivetrainSubsystem drivetrain,
            RobotState robotState) {
        super(xSupplier, ySupplier, rotationSupplier, fieldCentricSupplier, drivetrain);
        this.robotState = robotState;

        drive.HeadingController.setPID(3.5, 0, 0);
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Rotation2d calculateHeading() {
        return Rotation2d.fromRadians(
                AimingCalculator.aimToPoint(robotState.getFieldToRobot(), robotState.getChassisSpeeds(true),
                new Translation2d(
                    Units.inchesToMeters(8),
                    Units.inchesToMeters(218.415)))[1]);
    }

    @Override
    public SwerveRequest getSwerveRequest() {
        drive.withTargetDirection(calculateHeading())
                .withVelocityX(getX() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
                .withVelocityY(getY() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond());

        return drive;
    }
}
