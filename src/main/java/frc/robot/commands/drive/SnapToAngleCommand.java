// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class SnapToAngleCommand extends DefaultDriveCommand {
  private final RobotState robotState;

  private SwerveRequest.FieldCentricFacingAngle drive;

  private Rotation2d desiredDirection;

  public SnapToAngleCommand(
      Rotation2d desiredDirection,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier,
      DrivetrainSubsystem drivetrain,
      RobotState robotState) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentricSupplier, drivetrain);
    this.robotState = robotState;
    this.desiredDirection = desiredDirection;

    drive = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(DrivetrainSubsystem.getMaxVelocityMetersPerSecond() * 0.05)
    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    drive.HeadingController.setPID(3.5, 0, 0);
    drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    drive.HeadingController.setTolerance(Units.degreesToRadians(DrivetrainConstants.HEADING_TOLERANCE));
    Logger.recordOutput("Snap Direction ", desiredDirection.getDegrees());
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    drive.withTargetDirection(desiredDirection)
            .withVelocityX(getX() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
            .withVelocityY(getY() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond());

    return drive;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
