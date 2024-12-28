// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.testAutos;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoDescription;
import frc.robot.auto.modes.AutoBase;

@AutoDescription(description = "Test Auto c12")
public class TestAuto extends AutoBase {
  private static final PathPlannerPath pathc1 = getPathFromFile("C-1");
  private static final PathPlannerPath path12 = getPathFromFile("1-2");

  public TestAuto() {
    super(pathc1.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(createFollowPathCommand(pathc1)); // , createFollowPathCommand(path12));
  }
}
