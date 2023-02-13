package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.HashMap;
import java.util.LinkedHashMap;

@Config
public class ControllerTurret extends AutoTurret {
    private final GamepadEx gamepad;
    private final ControllerTools.BoxedBoolean doingstuff = new ControllerTools.BoxedBoolean();
    public static double ampltiude = 0.5;
    private final LinkedHashMap<ButtonReader, Double> turretButtons;
    private final HashMap<ButtonReader, Boolean> turretButtonVals = new HashMap<>();
    private double speed = 1.0;
    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(motor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public ControllerTurret(HardwareMap hardwareMap, GamepadEx gamepad) {
        super(hardwareMap);
        this.gamepad = gamepad;
        this.turretButtons = new LinkedHashMap<ButtonReader, Double>() {{
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP), 0D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT), 90D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN), 180D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT), 270D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER), 315D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER), 45D);
        }};
    }

    public void whopper() {
        ControllerTools.setPosFromButtonMap(turretButtonVals, turretButtons, doingstuff, (turPos) -> {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(speed);
            setPos(turPos, Units.DEGREES);
        });
        final double neg = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        final double pos = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (!doingstuff.value) {
            motor.setPower(Math.max(neg, pos) == neg ? -neg*ampltiude : pos*ampltiude);
        } else if (neg > 0.05 | pos > 0.05 || !isMoving()) {
            doingstuff.value = false;
            cleanup();
        }
    }
}
