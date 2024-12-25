// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DefaultDriveCommand extends Command {

  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldCentricSupplier;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotationLimiter;

  /**
   * @param xSupplier        supplier for forward velocity.
   * @param ySupplier        supplier for sideways velocity.
   * @param rotationSupplier supplier for angular velocity.
   */
  public DefaultDriveCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;

    xLimiter = new SlewRateLimiter(2);
    yLimiter = new SlewRateLimiter(2);
    rotationLimiter = new SlewRateLimiter(5);

    addRequirements(drivetrain);
  }

  protected double getX() {
    return slewAxis(xLimiter, xSupplier.getAsDouble());
  }

  protected double getY() {
    return slewAxis(yLimiter, ySupplier.getAsDouble());
  }

  protected double getRotation() {
    return slewAxis(rotationLimiter, rotationSupplier.getAsDouble());
  }

  protected boolean getFieldCentric() {
    return fieldCentricSupplier.getAsBoolean();
  }

  protected SwerveRequest getSwerveRequest() {
    if (getFieldCentric()) {
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(DrivetrainSubsystem.getMaxVelocityMetersPerSecond() * 0.05)
                .withRotationalDeadband(DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond() * 0.05) // Add a 5% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(getX() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
                .withVelocityY(getY() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
                .withRotationalRate(getRotation() * DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond());
        return drive;
    } else {
        SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDeadband(DrivetrainSubsystem.getMaxVelocityMetersPerSecond() * 0.05)
                .withRotationalDeadband(DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond() * 0.05) // Add a 5% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(getX() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
                .withVelocityY(getY() * DrivetrainSubsystem.getMaxVelocityMetersPerSecond())
                .withRotationalRate(getRotation() * DrivetrainSubsystem.getMaxAngularVelocityRadiansPerSecond());
        return drive;
    }
  }

  @Override
  public void execute() {
    drivetrain.setControl(getSwerveRequest());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  protected double slewAxis(SlewRateLimiter limiter, double value) {
    return limiter.calculate(Math.copySign(Math.pow(value, 2), value));
  }
}
