package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.common.AutoRequirements;

public abstract class AutoBase extends SequentialCommandGroup {
    protected final AutoRequirements autoRequirements;

    protected AutoBase(
            Pose2d startingPose,
            AutoRequirements autoRequirements) {
        this.autoRequirements = autoRequirements;

        setStartingPose(startingPose);
    }

    public abstract void init();

    private void setStartingPose(Pose2d startingPose) {
        addCommands(new InstantCommand(() -> autoRequirements.getDrivetrain().seedFieldRelative(startingPose)));
    }

    protected Command createFollowPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }
}
