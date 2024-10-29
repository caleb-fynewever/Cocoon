package frc.robot.controlboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.controlboard.primary.GamepadPrimaryInput;
import frc.robot.controlboard.primary.IPrimaryControlBoard;
import frc.robot.controlboard.primary.JoystickPrimaryInput;
import frc.robot.controlboard.secondary.ControlPanelInput;
import frc.robot.controlboard.secondary.ISecondaryControlBoard;
import frc.robot.util.Ports;

public class ControlBoard implements IPrimaryControlBoard, ISecondaryControlBoard {
    private static ControlBoard INSTANCE = null;

    public static ControlBoard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ControlBoard();
        }
        return INSTANCE;
    }

    private final IPrimaryControlBoard primaryControlBoard;
    private final ISecondaryControlBoard secondaryControlBoard;

    private ControlBoard() {
        boolean useDriveGamepad = DriverConstants.FORCE_GAMEPAD || DriverStation.getJoystickIsXbox(Ports.GAMEPAD_PORT);
        primaryControlBoard = useDriveGamepad ? GamepadPrimaryInput.getInstance() : JoystickPrimaryInput.getInstance();
        secondaryControlBoard = ControlPanelInput.getInstance();
    }

    /* Primary */

    @Override
    public double getThrottle() {
        return primaryControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return primaryControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return primaryControlBoard.getRotation();
    }

    @Override
    public Trigger pov() {
        return primaryControlBoard.pov();
    }

    @Override
    public double povVal() {
        return primaryControlBoard.povVal();
    }

    @Override
    public Trigger resetGyro() {
        return primaryControlBoard.resetGyro();
    }

    @Override
    public Trigger aimToGoal() {
        return primaryControlBoard.aimToGoal();
    }

    @Override
    public Trigger aimToAmp() {
        return primaryControlBoard.aimToAmp();
    }

    @Override
    public Trigger intake() {
        return primaryControlBoard.intake();
    }

    @Override
    public Trigger shoot() {
        return primaryControlBoard.shoot();
    }

    /* Secondary */

    @Override
    public Trigger climb() {
        return secondaryControlBoard.climb();
    }
}
