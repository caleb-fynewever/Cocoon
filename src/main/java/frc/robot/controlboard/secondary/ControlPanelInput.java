package frc.robot.controlboard.secondary;

public class ControlPanelInput implements ISecondaryControlBoard{
    private static ControlPanelInput INSTANCE = null;

    public static ControlPanelInput getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ControlPanelInput();
        }
        return INSTANCE;
    }
    
}
