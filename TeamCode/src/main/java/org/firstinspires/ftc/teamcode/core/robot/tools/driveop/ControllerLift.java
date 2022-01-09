package org.firstinspires.ftc.teamcode.core.robot.tools.driveop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedIndefinitelyEvent;

/**
 * lift and arm
 */
public class ControllerLift extends AutoLift {
    private final TriggerReader leftReader;
    private final TriggerReader rightReader;
    private final GamepadEx toolGamepad;
    private final DigitalChannel topSensor;

    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public ControllerLift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad) {
        super(eventThread, map);
        this.toolGamepad = toolGamepad;
        leftReader = new TriggerReader(toolGamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightReader = new TriggerReader(toolGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        topSensor = map.get(DigitalChannel.class,"topSensor");
        topSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void init() {
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(() -> setPosition(Positions.TOP), () -> {
            leftReader.readValue();
            return leftReader.wasJustReleased();
        }));
        eventThread.addEvent(new RunWhenOutputChangedIndefinitelyEvent(() -> setPosition(Positions.SAFE), () -> {
            rightReader.readValue();
            return rightReader.wasJustReleased();
        }));
    }

    @Override
    public void update() {
        super.update();
        if (position == Positions.SAFE && state == MovementStates.NONE) {
            if (liftMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (liftMotor.getCurrentPosition() >= 1375 && !topSensor.getState()) {
                liftMotor.setPower(toolGamepad.getLeftY());
            } else {
                liftMotor.setPower(0);
            }
        } else {
            if (liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}