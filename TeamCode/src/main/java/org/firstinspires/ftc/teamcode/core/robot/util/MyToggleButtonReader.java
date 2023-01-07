package org.firstinspires.ftc.teamcode.core.robot.util;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

/**
 * Class gets the current state of a toggle button
 */
public class MyToggleButtonReader extends BetterButtonReader {

    private boolean currToggleState;
    
    public MyToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button, startState) {
        super(gamepad, button);
        currToggleState = startState;
    }
            
    /**
     * The constructor that uses the gamepad and button to refer to a certain state toggler.
     *
     * @param gamepad the gamepad object that contains the buttonn
     * @param button  the button on the oject
     */
    public MyToggleButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        MyToggleButtonReader(gamepad, button, false);
    }

    /**
     * @return updates and returns the toggler
     */
    public boolean update() {
        if (wasJustReleased()) { // this is the sus part because it prbly reads but idk
            currToggleState = !currToggleState;
        }
        return currToggleState;
    }

}
