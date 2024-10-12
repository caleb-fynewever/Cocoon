// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team2052.generated.TunerConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Telemetry;

public class RobotContainer {
  public final Joystick driveJoystick;
  public final Joystick turnJoystick;
  public final Joystick controlPanel;

  public final DrivetrainSubsystem drivetrain;

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(DrivetrainSubsystem.getMaxVelocityMetersPerSecond());

  public RobotContainer() {
    driveJoystick = new Joystick(0);
    turnJoystick = new Joystick(1);
    controlPanel = new Joystick(2);
    drivetrain = new DrivetrainSubsystem(DrivetrainConstants, FrontLeft,
        FrontRight, BackLeft, BackRight);
    drivetrain.setDefaultCommand(
      new DefaultDriveCommand(
        driveJoystick::getY,
        // Sideways velocity supplier.
        driveJoystick::getX,
        // Rotation velocity supplier.
        turnJoystick::getX,
        () -> false,
        drivetrain);
    configureBindings();
  }

  private void configureBindings() {
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    drivetrain.registerTelemetry(logger::telemeterize);

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}