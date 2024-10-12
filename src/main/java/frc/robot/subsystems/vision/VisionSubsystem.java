package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.VisionConstants.Camera0Constants;
import frc.robot.Constants.VisionConstants.Camera1Constants;
import frc.robot.Constants.VisionConstants.Camera2Constants;
import frc.robot.Constants.VisionConstants.Camera3Constants;

public class VisionSubsystem extends SubsystemBase {
    private List<VisionUpdate> visionUpdates = new ArrayList<VisionUpdate>();
    private List<VisionUpdate> synchronizedVisionUpdates = Collections.synchronizedList(visionUpdates);

    private RobotState robotState;
    private List<TagTracker> cameras = new ArrayList<TagTracker>();

    public VisionSubsystem(RobotState robotState) {
        Collections.addAll(
                cameras,
                new TagTracker(Camera0Constants.TagTrackerConstants()),
                new TagTracker(Camera1Constants.TagTrackerConstants()),
                new TagTracker(Camera2Constants.TagTrackerConstants()),
                new TagTracker(Camera3Constants.TagTrackerConstants()));

        this.robotState = robotState;
    }

    private void pullCameraData(TagTracker camera) {
        Optional<VisionUpdate> visionUpdate = camera.getVisionUpdate();

        if (visionUpdate.isPresent()) {
            synchronizedVisionUpdates.add(visionUpdate.get());
        } else {
            // System.out.println("no vision update present for " + camera.getName());
        }
    }

    @Override
    public void periodic() {
        synchronizedVisionUpdates.clear();

        cameras.parallelStream().forEach(this::pullCameraData);

        synchronizedVisionUpdates.forEach(robotState::addVisionUpdate);
    }
}
