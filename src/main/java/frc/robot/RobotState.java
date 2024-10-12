package frc.robot;

public class RobotState {
    
    private static RobotState INSTANCE;
    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {

    }
}
