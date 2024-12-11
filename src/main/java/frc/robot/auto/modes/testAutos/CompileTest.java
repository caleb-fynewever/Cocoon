// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.testAutos;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.common.AutoRequirements;
import frc.robot.auto.modes.AutoBase;

@AutoDescription(description = "Compile/recompile test")
public class CompileTest extends AutoBase{

    private final static PathPlannerPath startPathCenterTo1 = PathPlannerPath.fromPathFile("StP-CS-1");
    
    public CompileTest(AutoRequirements autoRequirements){
        super(new Pose2d(),  autoRequirements);
    }

    @Override
    public void init(){
        new WaitCommand(3).execute();
        addCommands(createFollowPathCommand(startPathCenterTo1));
    }

}
