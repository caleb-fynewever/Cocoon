package frc.robot.controlboard.primary;

import com.team2052.lib.MathHelpers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.Ports;

public class GamepadPrimaryInput implements IPrimaryControlBoard{
    private final CommandXboxController controller;

    private static GamepadPrimaryInput INSTANCE;
    public static GamepadPrimaryInput getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GamepadPrimaryInput();
        }
        return INSTANCE;
    }

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
    public Trigger povUp() {
        return controller.povUp();
    }

    @Override
    public Trigger povUpRight() {
        return controller.povUpRight();
    }

    @Override
    public Trigger povRight() {
        return controller.povRight();
    }

    @Override
    public Trigger povDownRight() {
        return controller.povDownRight();
    }

    @Override
    public Trigger povDown() {
        return controller.povDown();
    }

    @Override
    public Trigger povDownLeft() {
        return controller.povDownLeft();
    }

    @Override
    public Trigger povLeft() {
        return controller.povLeft();
    }

    @Override
    public Trigger povUpLeft() {
        return controller.povUpLeft();
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

    @Override
    public Trigger intake() {
        return controller.leftTrigger(0.125);
    }

    @Override
    public Trigger shoot() {
        return controller.rightTrigger(0.125);
    }

    @Override
    public Trigger aimToGoal() {
        return controller.button(1);
    }

    @Override
    public Trigger aimToAmp() {
        return controller.button(2);
    }
}