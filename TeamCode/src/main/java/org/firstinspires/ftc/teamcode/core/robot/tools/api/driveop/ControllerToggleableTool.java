package org.firstinspires.ftc.teamcode.core.robot.tools.api.driveop;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.api.headless.HeadlessToggleableTool;
import org.firstinspires.ftc.teamcode.core.thread.EventHelper;
import org.firstinspires.ftc.teamcode.core.thread.event.impl.ReaderUpdatedEvent;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

/**
 * simple button push toggleable tool
 */
public abstract class ControllerToggleableTool<T extends HeadlessToggleableTool> {
    protected final ButtonReader reader;
    public final T headlessTool;

    /**
     * @param eventHelper  local instance of eventHelper
     * @param toolGamepad  instance of GamepadEx from FtcLib
     * @param button       button to be pushed for toggle, uses GamepadKeys.Button
     * @param headlessTool the headless version of this tool
     */
    public ControllerToggleableTool(@NonNull EventHelper eventHelper, GamepadEx toolGamepad, GamepadKeys.Button button, T headlessTool) {
        this.headlessTool = headlessTool;
        this.reader = new ButtonReader(toolGamepad, button);
        eventHelper.addEvent(new ReaderUpdatedEvent(this::run, reader));
    }

    private void run() {
        if (reader.wasJustReleased()) {
            if (headlessTool.isOn()) {
                headlessTool.off();
            } else {
                headlessTool.on();
            }
        }
    }
}
