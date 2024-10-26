package frc.robot.controlboard.primary;

import com.team2052.lib.MathHelpers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.Ports;

public class GamepadPrimaryInput implements IPrimaryControlBoard{
    
    private static GamepadPrimaryInput INSTANCE = null;

    public static GamepadPrimaryInput getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GamepadPrimaryInput();
        }
        return INSTANCE;
    }
    private final CommandXboxController controller;

    private GamepadPrimaryInput() {
        controller = new CommandXboxController(Ports.GAMEPAD_PORT);
    }

    @Override
    public double getThrottle() {
        return -MathHelpers.deadband(controller.getRawAxis(1), DriverConstants.GAMEPAD_DEADBAND);
    }

    @Override
    public double getStrafe() {
        return -MathHelpers.deadband(controller.getRawAxis(0), DriverConstants.GAMEPAD_DEADBAND);
    }

    @Override
    public double getRotation() {
        return -MathHelpers.deadband(controller.getRawAxis(2), DriverConstants.GAMEPAD_DEADBAND);
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

    @Override
    public Trigger aimToGoal() {
        return controller.button(1);
    }

    @Override
    public Trigger intake() {
        return controller.leftTrigger(0.125);
    }

    @Override
    public Trigger shoot() {
        return controller.rightTrigger(0.125);
    }
}