package frc.robot.util.io;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.common.AutoFactory.Auto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Dashboard {
    private final LoggedDashboardChooser<DriveMode> driveModeChooser = new LoggedDashboardChooser<>("Drive Mode");

    private final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    public Dashboard() {
        driveModeChooser.addDefaultOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.FIELD_CENTRIC.name(), DriveMode.FIELD_CENTRIC);
        driveModeChooser.addOption(DriveMode.ROBOT_CENTRIC.name(), DriveMode.ROBOT_CENTRIC);

        autoChooser.addDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);
        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
    }

    public <V> void putData(String key, V value) {
        if (value instanceof Float) {
            SmartDashboard.putNumber(key, (Float) value);
        } else if (value instanceof Integer) {
            SmartDashboard.putNumber(key, (Integer) value);
        } else if (value instanceof Number) {
            SmartDashboard.putNumber(key, (Double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (Boolean) value);
        } else if (value instanceof Sendable) {
            Shuffleboard.getTab("main").add(key, (Sendable) value);
        }
    }

    public boolean isFieldCentric() {
        return driveModeChooser.get() == DriveMode.FIELD_CENTRIC;      
    }

    public Auto getAuto() {
        return autoChooser.get();
    }

    // Enums for Dashboard elements:
    public static enum DriveMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC;
    }
}
