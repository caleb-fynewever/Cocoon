// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.testAutos;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.modes.AutoBase;

@AutoDescription(description = "Center to close 3 pickup, amp side 8 + 7")
public class NewPPAuto12387 extends AutoBase {
    private final static PathPlannerPath startPathCenterTo1 = getPathFromFile("StP-CS-1");
    private final static PathPlannerPath pickUpPathS1To2 = getPathFromFile("PiPS1-2");
    private final static PathPlannerPath pickUpPathS2To3 = getPathFromFile("PipS-2-3");
    private final static PathPlannerPath pickUpPathS3To8 = getPathFromFile("PiPS-3-8");
    private final static PathPlannerPath scorePath8toAS = getPathFromFile("ScP-8-AS");
    private final static PathPlannerPath pickUpPathASto7 = getPathFromFile("PiP-AS-7");
    private final static PathPlannerPath scorePath7ToAS = getPathFromFile("ScP 7-AS");

    public NewPPAuto12387() {
        super(startPathCenterTo1.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(createFollowPathCommand(startPathCenterTo1));
        addCommands(createFollowPathCommand(pickUpPathS1To2));
        addCommands(createFollowPathCommand(pickUpPathS2To3));
        addCommands(createFollowPathCommand(pickUpPathS3To8));
        addCommands(createFollowPathCommand(scorePath8toAS));
        addCommands(createFollowPathCommand(pickUpPathASto7));
        addCommands(createFollowPathCommand(scorePath7ToAS));
    }

}
