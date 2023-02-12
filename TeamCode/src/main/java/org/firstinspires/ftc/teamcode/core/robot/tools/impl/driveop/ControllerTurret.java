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
import java.util.Map;
import java.util.Objects;

@Config
public class ControllerTurret extends AutoTurret {
    private final GamepadEx gamepad;
    private final ControllerTools.BoxedBoolean doingstuff = new ControllerTools.BoxedBoolean();
    public static double ampltiude = 0.5;
    private final LinkedHashMap<ButtonReader, AutoTurret.Rotation> turretButtons;
    private final HashMap<ButtonReader, Boolean> turretButtonVals = new HashMap<>();
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
        this.turretButtons = new LinkedHashMap<ButtonReader, Rotation>() {{
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP), Rotation.FRONT);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT), Rotation.RIGHT);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN), Rotation.BACK);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT), Rotation.LEFT);
            put(new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER), Rotation.FRONTLEFT);
            put(new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER), Rotation.FRONTRIGHT);
        }};
    }

    public void update() {
        ControllerTools.setPosFromButtonMap(turretButtonVals, turretButtons, doingstuff, (turPos) -> {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            setPos(turPos);
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
