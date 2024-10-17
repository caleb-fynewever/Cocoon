// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Add your docs here. */
public class AutoRequirements {
    private final RobotState robotState;
    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem vision;

    public AutoRequirements(
        RobotState robotState,
        DrivetrainSubsystem drivetrain,
        VisionSubsystem vision
    ) {
        this.robotState = robotState;
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public VisionSubsystem getVision() {
        return vision;
    }
}
