package frc.robot.auto.modes;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public abstract class AutoBase extends SequentialCommandGroup {
    private final RobotState robotState = RobotState.getInstance();
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private Pose2d startingPose;

    protected AutoBase(Optional<Pose2d> pathStartingPose) {

        if(pathStartingPose.isEmpty()) {
            startingPose = new Pose2d();
        } else {
            startingPose = pathStartingPose.get();
        }

        if (RobotState.getInstance().isRedAlliance()) {
            startingPose = FlippingUtil.flipFieldPose(startingPose);
        }
        setStartingPose(startingPose);
        Logger.recordOutput("Red Alliance", this.robotState.isRedAlliance());
        Logger.recordOutput("Auto Starting Pose", startingPose);
    }

    public abstract void init();

    private void setStartingPose(Pose2d pathStartingPose) {
        addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartingPose)));
    }

    protected Command createFollowPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }
    
    protected static PathPlannerPath getPathFromFile(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return path;
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }
}
