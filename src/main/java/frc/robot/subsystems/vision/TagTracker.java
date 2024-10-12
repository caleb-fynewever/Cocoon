package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class TagTracker {
    private PhotonPoseEstimator poseEstimator;
    public PhotonCamera photonCamera;

    public TagTracker(TagTrackerConstants camConstants) {
        photonCamera = new PhotonCamera(camConstants.name);

        poseEstimator = new PhotonPoseEstimator(camConstants.tagLayout, camConstants.strategy, photonCamera, camConstants.robotToCamera);
    }

    public String getName() {
        return photonCamera.getName();
    }

    public Optional<VisionUpdate> getVisionUpdate() {
        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
        if(estimatedRobotPose.isPresent()){
            return Optional.of(new VisionUpdate(this.photonCamera.getName(), estimatedRobotPose.get(), null));
        } else {
            return Optional.empty();
        }
    } 

    public static class TagTrackerConstants {
        public final String name;
        public final Transform3d robotToCamera;
        public final AprilTagFieldLayout tagLayout;
        public final PoseStrategy strategy;

        public TagTrackerConstants(String name, Transform3d robotToCamera, AprilTagFieldLayout tayLayout, PoseStrategy strategy) {
            this.name = name;
            this.robotToCamera = robotToCamera;
            this.tagLayout = tayLayout;
            this.strategy = strategy;
        }
    }
}
