package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOHardware;

public class VisionSubsystem extends SubsystemBase implements VisionIO {
    private VisionIO vision;
    private RobotState robotState = RobotState.getInstance();

    public static VisionSubsystem INSTANCE;
    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }

        return INSTANCE;
    }
    private VisionSubsystem() {
        if(Robot.isSimulation()) {
            //vision = new VisionIOSim(robotState);
        } else { 
            vision = new VisionIOHardware(robotState);
        }
    }

    @Override
    public void periodic() {
        synchronizedVisionUpdates.clear();
        
        update();

        synchronizedVisionUpdates.forEach(robotState::addVisionUpdate);
    }

    @Override
    public void update() {
        vision.update();
    }
}
