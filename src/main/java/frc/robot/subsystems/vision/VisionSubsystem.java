package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;

public class VisionSubsystem extends SubsystemBase implements IVisionSubsystem {
    private IVisionSubsystem vision;
    private RobotState robotState;

    public VisionSubsystem(RobotState robotState) {
        this.robotState = robotState;
        if(Robot.isSimulation()) {
            vision = new VisionSim(robotState);
        } else { 
            vision = new VisionHardware(robotState);
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
