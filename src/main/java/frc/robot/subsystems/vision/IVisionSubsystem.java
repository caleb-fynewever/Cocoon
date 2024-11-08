package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface IVisionSubsystem {

    List<VisionUpdate> visionUpdates = new ArrayList<VisionUpdate>();
    List<VisionUpdate> synchronizedVisionUpdates = Collections.synchronizedList(visionUpdates);
    List<TagTracker> cameras = new ArrayList<TagTracker>();
    void update();
}
