package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOHardware;
import frc.robot.subsystems.vision.io.VisionIOSim;

public class VisionSubsystem extends SubsystemBase implements VisionIO {
    private VisionIO vision;
    private RobotState robotState;

    public VisionSubsystem(RobotState robotState) {
        this.robotState = robotState;
        if(Robot.isSimulation()) {
            vision = new VisionIOSim(robotState);
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
