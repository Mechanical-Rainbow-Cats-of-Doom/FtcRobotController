package org.firstinspires.ftc.teamcode.core.robot.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.BooleanSupplier;

/**
 * Class gets the current state of a toggle button
 */
public class MyToggleButtonReader extends BetterButtonReader {

    public boolean currToggleState;

    /**
     * The constructor that uses the gamepad and button to refer to a certain state toggler.
     *
     * @param gamepad the gamepad object that contains the buttonn
     * @param button  the button on the oject
     */
    public MyToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        super(gamepad, button);
        currToggleState = false;
    }

    /**
     * @return the current state of the toggler
     */
    public boolean getState() {
        if (wasJustReleased()) {
            currToggleState = !currToggleState;
        }
        return (currToggleState);
    }

}
