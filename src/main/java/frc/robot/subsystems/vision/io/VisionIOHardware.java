package frc.robot.subsystems.vision.io;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import java.util.Collections;

import frc.robot.Constants.VisionConstants.Camera0Constants;
import frc.robot.Constants.VisionConstants.Camera1Constants;
import frc.robot.Constants.VisionConstants.Camera2Constants;
import frc.robot.Constants.VisionConstants.Camera3Constants;
import frc.robot.subsystems.vision.TagTracker;
import frc.robot.subsystems.vision.VisionUpdate;
import frc.robot.RobotState;

public class VisionIOHardware implements VisionIO {
    public VisionIOHardware(RobotState robotState) {
        Collections.addAll(
                cameras,
                new TagTracker(Camera0Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera1Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera2Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera3Constants.TagTrackerConstants(), robotState));
    }

    public void update() {
        cameras.parallelStream().forEach(this::pullCameraData);
    }

    public void pullCameraData(TagTracker tagTracker) {
        Optional<VisionUpdate> visionUpdate = tagTracker.getVisionUpdate();

        if (visionUpdate.isPresent()) {
            synchronizedVisionUpdates.add(visionUpdate.get());
            Logger.recordOutput(tagTracker.getName() + " has update", true);
        } else {
            Logger.recordOutput(tagTracker.getName() + " has update", false);
        }
    }
}
