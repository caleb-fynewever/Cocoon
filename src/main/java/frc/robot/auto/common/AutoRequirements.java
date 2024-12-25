// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

/** Add your docs here. */
public class AutoRequirements {
    private final RobotState robotState = RobotState.getInstance();
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    public AutoRequirements() {}

    public RobotState getRobotState() {
        return robotState;
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }
}
