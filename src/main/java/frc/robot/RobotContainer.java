// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.BackLeftModule;
import frc.robot.Constants.DrivetrainConstants.BackRightModule;
import frc.robot.Constants.DrivetrainConstants.FrontLeftModule;
import frc.robot.Constants.DrivetrainConstants.FrontRightModule;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionUpdate;
import frc.robot.util.Telemetry;

public class RobotContainer {
  public final Joystick driveJoystick;
  public final Joystick turnJoystick;
  public final Joystick controlPanel;

  public final DrivetrainSubsystem drivetrain;

  private final Consumer<VisionUpdate> visionEstimateConsumer = new Consumer<VisionUpdate>() {
    @Override
    public void accept(VisionUpdate estimate) {
      drivetrain.addVisionMeasurement(estimate);
    }
  };
  
  private final RobotState robotState = new RobotState(visionEstimateConsumer);

  private final Telemetry logger = new Telemetry(DrivetrainSubsystem.getMaxVelocityMetersPerSecond());

  public RobotContainer() {
    driveJoystick = new Joystick(0);
    turnJoystick = new Joystick(1);
    controlPanel = new Joystick(2);
    
    drivetrain = new DrivetrainSubsystem(
        DrivetrainConstants.DrivetrainConstants,
        FrontLeftModule.getConstants(),
        FrontRightModule.getConstants(),
        BackLeftModule.getConstants(),
        BackRightModule.getConstants());

    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            driveJoystick::getY,
            // Sideways velocity supplier.
            driveJoystick::getX,
            // Rotation velocity supplier.
            turnJoystick::getX,
            () -> false,
            drivetrain,
            robotState));
    configureBindings();
  }

  private void configureBindings() {

    drivetrain.registerTelemetry(logger::telemeterize);

    /* Bindings for drivetrain characterization */
    /*
     * These bindings require multiple buttons pushed to swap between quastatic and
     * dynamic
     */
    /*
     * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
     */
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}