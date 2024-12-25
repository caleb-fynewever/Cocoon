// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.testAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.modes.AutoBase;

@AutoDescription(description = "Compile/recompile test")
public class CompileTest extends AutoBase {

  private static final PathPlannerPath startPathCenterTo1 = getPathFromFile("StP-CS-1");

  public CompileTest() {
    super(startPathCenterTo1.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    new WaitCommand(3).execute();
    addCommands(createFollowPathCommand(startPathCenterTo1));
  }
}
