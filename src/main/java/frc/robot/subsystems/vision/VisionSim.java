package frc.robot.subsystems.vision;

import java.util.Collections;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Camera0Constants;
import frc.robot.Constants.VisionConstants.Camera1Constants;
import frc.robot.Constants.VisionConstants.Camera2Constants;
import frc.robot.Constants.VisionConstants.Camera3Constants;
import frc.robot.RobotState;

public class VisionSim implements IVisionSubsystem {
    private VisionSystemSim visionSim;
    private SimCameraProperties simCamProperties;

    private final int resWidth = 1280;
    private final int resHeight = 800;

    private RobotState robotState;

    public VisionSim(RobotState robotState) {
        this.robotState = robotState;
        Collections.addAll(
                cameras,
                new TagTracker(Camera0Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera1Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera2Constants.TagTrackerConstants(), robotState),
                new TagTracker(Camera3Constants.TagTrackerConstants(), robotState));
        visionSim = new VisionSystemSim("VisionSubsystemSim");

        visionSim.addAprilTags(VisionConstants.APRIL_TAG_FIELD_LAYOUT);

        simCamProperties = new SimCameraProperties();
        simCamProperties.setCalibration(resWidth, resHeight, Rotation2d.fromDegrees(97.7));
        simCamProperties.setCalibError(0.35, 0.10);
        simCamProperties.setFPS(15);
        simCamProperties.setAvgLatencyMs(20);
        simCamProperties.setLatencyStdDevMs(5);

        cameras.forEach(this::addSimCameras);
    }

    public void addSimCameras(TagTracker tagTracker) {
        PhotonCameraSim camSim = new PhotonCameraSim(tagTracker.photonCamera, simCamProperties);
        visionSim.addCamera(camSim, tagTracker.constants.robotToCamera);
    }

    public void update() {
        EstimatedRobotPose estimatedRobotPose = new EstimatedRobotPose(visionSim.getRobotPose(), Timer.getFPGATimestamp(), null, null);
        // synchronizedVisionUpdates.add(new VisionUpdate("Vision Sim", estimatedRobotPose, robotState.getFieldToRobot()));
    }
}
