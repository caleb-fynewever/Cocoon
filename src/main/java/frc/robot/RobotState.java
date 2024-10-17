package frc.robot;

import java.util.function.Consumer;

import com.team254.lib.util.MathHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.VisionUpdate;

public class RobotState {

    private final static double LOOKBACK_TIME = 1.0;
    private Pose2d fieldToRobot = MathHelpers.kPose2dZero;
    private final TimeInterpolatableBuffer<Pose2d> odometryMeasurements = TimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    
    private final Consumer<VisionUpdate> visionUpdateConsumer;

    public RobotState(Consumer<VisionUpdate> visionUpdateConsumer) {
        this.visionUpdateConsumer = visionUpdateConsumer;
        // Make sure to protect callers against null.
        fieldToRobot = MathHelpers.kPose2dZero;
    }

    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }

    public void addFieldToRobot(Pose2d fieldToRobot) {
        this.fieldToRobot = fieldToRobot;
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        odometryMeasurements.addSample(timestamp, pose);
    }

    public void addVisionUpdate(VisionUpdate visionUpdate) {
        visionUpdateConsumer.accept(visionUpdate);
    }
    
    /**
     * Returns true if the robot is on red alliance.
     *
     * @return True if the robot is on red alliance.
     */
    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }
}
