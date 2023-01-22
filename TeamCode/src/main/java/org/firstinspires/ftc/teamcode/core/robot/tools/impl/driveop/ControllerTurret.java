package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;
@Config
public class ControllerTurret extends AutoTurret {
    private final GamepadEx gamepad;
    private boolean flip = false;
    private boolean doingstuff = false;
    private final ButtonReader leftBump, rightBump;
    public static double ampltiude = 0.5;
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
        this.leftBump = new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        this.rightBump = new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    public void update() {
        if (!doingstuff) {
            leftBump.readValue();
            rightBump.readValue();
            if (leftBump.wasJustReleased() || rightBump.wasJustReleased()) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
                if (leftBump.wasJustReleased()) {
                    setPos(getPos(Units.DEGREES) + (180 * (flip ? ampltiude : -ampltiude)), Units.DEGREES);
                }
                else setPos(0, Units.MOTOR_TICKS);
                flip = !flip;
                doingstuff = true;
            } else {
                final double neg = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                final double pos = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                motor.setPower(Math.max(neg, pos) == neg ? -neg : pos);
            }
        } else {
            if (!isMoving()) {
                doingstuff = false;
                cleanup();
            }
        }
    }
}
