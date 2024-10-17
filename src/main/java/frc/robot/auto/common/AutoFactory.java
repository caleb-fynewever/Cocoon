// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.auto.modes.AutoBase;
import frc.robot.auto.modes.testAutos.Sauce123Auto;
import frc.robot.util.io.Dashboard;

/**
 * Responsible for selecting, compiling, and recompiling autos before the start of a match.
 */
public class AutoFactory {
    private final Supplier<Auto> autoSupplier;
    private final AutoRequirements autoRequirements;

    private Auto currentAuto;
    
    private AutoBase compiledAuto;

    public AutoFactory(
        Supplier<Auto> autoSupplier,
        AutoRequirements autoRequirements
    ) {
        this.autoSupplier = autoSupplier;
        this.autoRequirements = autoRequirements;
    }

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto;
    }

    public void recompile() {
        Dashboard.getInstance().putData(DashboardConstants.AUTO_COMPILED_KEY, false);
        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }

        compiledAuto = currentAuto.getInstance(autoRequirements);

        if (compiledAuto != null) {
            compiledAuto.init();
        }

        Dashboard.getInstance().putData(DashboardConstants.AUTO_COMPILED_KEY, true);
    }

    public AutoBase getCompiledAuto() {
        return compiledAuto;
    }

    public static enum Auto {
        NO_AUTO(null),
        SAUCE_AUTO(Sauce123Auto.class);

        private final Class<? extends AutoBase> autoClass;

        private Auto(Class<? extends AutoBase> autoClass) {
            this.autoClass = autoClass;
        }

        public AutoBase getInstance(AutoRequirements autoRequirements) {
            if (autoClass != null) {
                try {

                    AutoDescription autoDescription = autoClass.getClass().getAnnotation(AutoDescription.class);
                    if(autoDescription != null){
                        Dashboard.getInstance().putData(Constants.DashboardConstants.AUTO_DESCRIPTION_KEY, autoDescription.description());
                    }

                    return autoClass.getConstructor(AutoRequirements.class).newInstance(autoRequirements);
                    
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            
            return null;
        }
    }
}
