package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;

public class TagTracker {
    private PhotonPoseEstimator poseEstimator;
    public PhotonCamera photonCamera;
    public TagTrackerConstants constants;
    public RobotState robotState;

    public TagTracker(TagTrackerConstants camConstants, RobotState robotState) {
        this.constants = camConstants;
        this.photonCamera = new PhotonCamera(camConstants.name);
        this.poseEstimator = new PhotonPoseEstimator(camConstants.tagLayout, camConstants.strategy, camConstants.robotToCamera);
        this.robotState = robotState;
    }

    public String getName() {
        return photonCamera.getName();
    }

    public Optional<VisionUpdate> getResultToPose(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
        if(estimatedRobotPose.isPresent()){
            return Optional.of(new VisionUpdate(this.photonCamera.getName(), estimatedRobotPose.get(), robotState.getFieldToRobot()));
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
