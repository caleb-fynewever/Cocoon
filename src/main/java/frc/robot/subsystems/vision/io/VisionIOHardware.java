package frc.robot.subsystems.vision.io;

import frc.robot.Constants.VisionConstants.Camera0Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.TagTracker;
import frc.robot.subsystems.vision.VisionUpdate;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOHardware implements VisionIO {
  public VisionIOHardware(RobotState robotState) {
    Collections.addAll(cameras, new TagTracker(Camera0Constants.TagTrackerConstants(), robotState));
  }

  public void update() {
    cameras.parallelStream().forEach(this::pullCameraData);
  }

  public void pullCameraData(TagTracker tagTracker) {
    List<PhotonPipelineResult> latestResults = tagTracker.photonCamera.getAllUnreadResults();
    for (int i = 0; i < latestResults.size(); i++) {
      Optional<VisionUpdate> visionUpdate = tagTracker.getResultToPose(latestResults.get(i));

      if (visionUpdate.isPresent()) {
        synchronizedVisionUpdates.add(visionUpdate.get());
        Logger.recordOutput(tagTracker.getName() + " has update", true);
      } else {
        Logger.recordOutput(tagTracker.getName() + " has update", false);
      }
    }
  }
}
