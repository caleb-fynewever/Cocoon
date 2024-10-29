package frc.robot.controlboard.primary;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IPrimaryControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();
    
    Trigger pov();

    double povVal();

    Trigger resetGyro();

    // Real Controls
    Trigger intake();

    Trigger shoot();

    Trigger aimToGoal();

    Trigger aimToAmp();
}