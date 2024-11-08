package frc.robot.controlboard.primary;

import com.team2052.lib.MathHelpers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.Ports;

public class JoystickPrimaryInput implements IPrimaryControlBoard {
    private static JoystickPrimaryInput INSTANCE = null;

    public static JoystickPrimaryInput getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new JoystickPrimaryInput();
        }
        return INSTANCE;
    }
    
    private final Joystick translateStick;
    private final Joystick rotateStick;
    
    private JoystickPrimaryInput() {
        translateStick = new Joystick(Ports.TRANSLATION_JOYSTICK_PORT);
        rotateStick = new Joystick(Ports.ROTATION_JOYSTICK_PORT);
    }


    @Override
    public double getThrottle() {
        return MathHelpers.deadband(translateStick.getRawAxis(1), DriverConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public double getStrafe() {
        return MathHelpers.deadband(translateStick.getRawAxis(0), DriverConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotation() {
        return MathHelpers.deadband(-rotateStick.getRawAxis(0), DriverConstants.JOYSTICK_DEADBAND);
    }

    @Override
    public Trigger povUp() {
        return new Trigger(rotateStick.povUp(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povUpRight() {
        return new Trigger(rotateStick.povUpRight(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povRight() {
        return new Trigger(rotateStick.povRight(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povDownRight() {
        return new Trigger(rotateStick.povDownRight(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povDown() {
        return new Trigger(rotateStick.povDown(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povDownLeft() {
        return new Trigger(rotateStick.povDownLeft(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povLeft() {
        return new Trigger(rotateStick.povLeft(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger povUpLeft() {
        return new Trigger(rotateStick.povUpLeft(povLooper)::getAsBoolean);
    }

    @Override
    public Trigger resetGyro() {
        return new JoystickButton(rotateStick, 11);
    }

    @Override
    public Trigger intake() {
        return new JoystickButton(translateStick, 1);
    }

    @Override
    public Trigger shoot() {
        return new JoystickButton(rotateStick, 1);
    }

    @Override
    public Trigger aimToGoal() {
        return new JoystickButton(rotateStick, 3);
    }

    @Override
    public Trigger aimToAmp() {
        return new JoystickButton(rotateStick, 4);
    }
}
