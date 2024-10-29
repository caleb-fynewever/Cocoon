package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Ports;

public class ControlPanelInput implements ISecondaryControlBoard{
    private static ControlPanelInput INSTANCE = null;

    public static ControlPanelInput getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ControlPanelInput();
        }
        return INSTANCE;
    }

    private final Joystick controlPanel;

    private ControlPanelInput() {
        controlPanel = new Joystick(Ports.CONTROL_PANEL_PORT);
    }
    
    @Override
    public Trigger climb() {
        return new JoystickButton(controlPanel, 1);
    }
}
