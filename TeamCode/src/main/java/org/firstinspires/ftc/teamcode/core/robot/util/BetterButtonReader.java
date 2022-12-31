package org.firstinspires.ftc.teamcode.core.robot.util;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class BetterButtonReader extends ButtonReader {
    public BetterButtonReader(GamepadEx gamepad, GamepadKeys.Button button) {
        super(gamepad, button);
    }

    @Override
    public boolean wasJustReleased() {
        boolean ret = super.wasJustReleased();
        readValue();
        return ret;
    }
}
