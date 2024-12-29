package frc.robot.subsystems.vision.io;

import frc.robot.subsystems.vision.TagTracker;
import frc.robot.subsystems.vision.VisionUpdate;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface VisionIO {

  List<VisionUpdate> visionUpdates = new ArrayList<VisionUpdate>();
  List<VisionUpdate> synchronizedVisionUpdates = Collections.synchronizedList(visionUpdates);
  List<TagTracker> cameras = new ArrayList<TagTracker>();

  void update();
}
