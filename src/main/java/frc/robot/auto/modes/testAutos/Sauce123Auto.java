// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.testAutos;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.modes.AutoBase;

@AutoDescription(description = "saucy 123 like a bbq on july fourth")
public class Sauce123Auto extends AutoBase{
    private final static PathPlannerPath startPathCenterTo1 = PathPlannerPath.fromPathFile("Path-C-1");
    private final static PathPlannerPath path1To2 = PathPlannerPath.fromPathFile("Path-1-2");
    private final static PathPlannerPath path2To3 = PathPlannerPath.fromPathFile("Path-2-3");
    
    public Sauce123Auto(AutoRequirements autoRequirements){
        super(startPathCenterTo1.getPreviewStartingHolonomicPose(), autoRequirements);
    }

    @Override
    public void init(){
        addCommands(createFollowPathCommand(startPathCenterTo1),
        createFollowPathCommand(path1To2),
        createFollowPathCommand(path2To3));
    }

}
