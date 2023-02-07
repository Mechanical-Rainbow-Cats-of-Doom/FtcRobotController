package org.firstinspires.ftc.teamcode.core.robot.util;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.BooleanSupplier;

/**
 * Class gets the current state of a toggle button
 */
public class ToggleableToggleButtonReader extends ButtonReader {

    private boolean currToggleState;

    /**
     * The constructor that uses the gamepad and button to refer to a certain state toggler.
     *
     * @param gamepad the gamepad object that contains the buttonn
     * @param button  the button on the oject
     */
    public ToggleableToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button, boolean currToggleState) {
        super(gamepad, button);

        this.currToggleState = currToggleState;
    }
    public ToggleableToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        this(gamepad, button, false);
    }

    /**
     * The constructor that checks the values returned by a boolean supplier
     * object.
     *
     * @param buttonValue the value supplier
     */
    public ToggleableToggleButtonReader(BooleanSupplier buttonValue) {
        super(buttonValue);

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

    public void forceToggle() {
        currToggleState = !currToggleState;
    }

    public void forceVal(boolean val) {
        currToggleState = val;
    }
}

