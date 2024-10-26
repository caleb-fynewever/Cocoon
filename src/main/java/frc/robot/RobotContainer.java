// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.commands.drive.AimChassisToGoalCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionUpdate;
import frc.robot.util.Telemetry;
import frc.robot.util.io.Dashboard;

public class RobotContainer {

  private final ControlBoard controlBoard = ControlBoard.getInstance();

  public final Dashboard dashboard;

  public final DrivetrainSubsystem drivetrain;
  //public final VisionSubsystem vision;

  public final AutoFactory autoFactory;

  private final Consumer<VisionUpdate> visionEstimateConsumer = new Consumer<VisionUpdate>() {
    @Override
    public void accept(VisionUpdate estimate) {
      drivetrain.addVisionMeasurement(estimate);
    }
  };

  public final RobotState robotState = new RobotState(visionEstimateConsumer);

  private final Telemetry logger = new Telemetry(DrivetrainSubsystem.getMaxVelocityMetersPerSecond());

  public RobotContainer() {

    dashboard = new Dashboard();

    drivetrain = new DrivetrainSubsystem(
        robotState,
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS,
        DrivetrainConstants.TUNER_MODULE_CONSTANTS);

    //vision = new VisionSubsystem(robotState);

    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            controlBoard::getThrottle,
            // Sideways velocity supplier.
            controlBoard::getStrafe,
            // Rotation velocity supplier.
            controlBoard::getRotation,
            dashboard::isFieldCentric,
            drivetrain));

    autoFactory = new AutoFactory(
        () -> dashboard.getAuto(),
        new AutoRequirements(
            robotState,
            drivetrain));

    configureBindings();
  }

  private void configureBindings() {

    drivetrain.registerTelemetry(logger::telemeterize);

    controlBoard.aimToGoal().whileTrue(new AimChassisToGoalCommand(controlBoard::getThrottle, controlBoard::getStrafe,
        controlBoard::getRotation, dashboard::isFieldCentric, drivetrain, robotState));

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

  public void forceRecompile() {
    autoFactory.recompile();
  }

  public void precompileAuto() {
    if (autoFactory.recompileNeeded()) {
      autoFactory.recompile();
    }
  }

  public Command getAutonomousCommand() {
    return autoFactory.getCompiledAuto();
  }
}