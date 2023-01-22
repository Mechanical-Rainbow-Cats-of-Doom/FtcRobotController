package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

public class ControllerTurret extends AutoTurret {
    private final GamepadEx gamepad;
    private boolean flip = false;
    private boolean doingstuff = false;
    private final ButtonReader aReader;
    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(motor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public ControllerTurret(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        super(hardwareMap);
        this.gamepad = toolGamepad;
        this.aReader = new ButtonReader(gamepad, GamepadKeys.Button.A);
    }

    public void update() {
        if (!doingstuff) {
            aReader.readValue();
            if (aReader.wasJustReleased()) {
                final double neg = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                final double pos = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                motor.setPower(Math.max(neg, pos) == neg ? -neg : pos);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPos(getPos(Units.DEGREES) + (180 * (flip ? 1 : -1)), Units.DEGREES);
                flip = !flip;
                doingstuff = true;
            }
        } else {
            if (!isMoving()) {
                doingstuff = false;
                cleanup();
            }
        }
    }
}
