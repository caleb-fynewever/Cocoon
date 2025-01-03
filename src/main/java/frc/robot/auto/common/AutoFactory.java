// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import frc.robot.Constants.DashboardConstants;
import frc.robot.auto.modes.AutoBase;
import frc.robot.auto.modes.testAutos.TestAuto;
import frc.robot.util.io.Dashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/** Responsible for selecting, compiling, and recompiling autos before the start of a match. */
public class AutoFactory {
  private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();

  private static LoggedNetworkBoolean autoCompiled =
      new LoggedNetworkBoolean(DashboardConstants.AUTO_COMPILED_KEY, false);
  private static LoggedNetworkString loggedAutoDescription =
      new LoggedNetworkString(DashboardConstants.AUTO_DESCRIPTION_KEY, "No Description");

  private Auto currentAuto;
  private AutoBase compiledAuto;

  private static AutoFactory INSTANCE;

  public static AutoFactory getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AutoFactory();
    }

    return INSTANCE;
  }

  private AutoFactory() {}

  public boolean recompileNeeded() {
    return autoSupplier.get() != currentAuto;
  }

  public void recompile() {
    autoCompiled.set(false);
    currentAuto = autoSupplier.get();
    if (currentAuto == null) {
      currentAuto = Auto.NO_AUTO;
    }

    compiledAuto = currentAuto.getInstance();

    if (compiledAuto != null) {
      compiledAuto.init();
    } else {
      loggedAutoDescription.set("No Auto Selected");
    }

    autoCompiled.set(true);
  }

  public AutoBase getCompiledAuto() {
    return compiledAuto;
  }

  public static enum Auto {
    NO_AUTO(null),
    TEST_AUTO(TestAuto.class);

    private final Class<? extends AutoBase> autoClass;

    private Auto(Class<? extends AutoBase> autoClass) {
      this.autoClass = autoClass;
    }

    public AutoBase getInstance() {
      if (autoClass != null) {
        try {
          loggedAutoDescription.set(autoClass.getAnnotation(AutoDescription.class).description());

          return autoClass.getConstructor().newInstance();
        } catch (Exception e) {
          e.printStackTrace();
        }
      }

      return null;
    }
  }
}
