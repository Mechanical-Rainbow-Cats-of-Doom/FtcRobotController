package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import java.util.Timer;

@Config
public class ControllerTools extends AutoTools {
    public static double armZeroPower = 0.075, liftZeroPower = 0.001;
    public double test;
    private final GamepadEx gamepad;
    private final ControllerTurret rotation;
    private final Telemetry telemetry;
    private final ToggleableToggleButtonReader xReader, yReader;
    private final ButtonReader bReader, up, right, down, left_dpad;
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final HashMap<ButtonReader, Position> liftButtons;
    private final HashMap<ButtonReader, Boolean> liftButtonVals = new HashMap<>();
    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public ControllerTools(HardwareMap hardwareMap, Timer timer, ControllerTurret rotation, GamepadEx toolGamepad, Telemetry telemetry) {
        super(hardwareMap, timer, rotation);
        isAuto = false;
        this.rotation = rotation;
        gamepad = toolGamepad;
        this.telemetry = telemetry;
        this.xReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.X);
        this.yReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.Y);
        this.bReader = new ButtonReader(gamepad, GamepadKeys.Button.B);
        this.up = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP);
        this.right = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        this.down = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN);
        this.left_dpad = new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT);
        this.liftButtons = new HashMap<ButtonReader, Position>() {{
            put(up, Position.HIGH_TARGET_NODUMP);
            put(down, Position.GROUND_TARGET_NODUMP);
            put(left_dpad, Position.LOW_TARGET_NODUMP);
            put(right, Position.MEDIUM_TARGET_NODUMP);
        }};
    }

    private void readLiftButtons() {
        for (ButtonReader button : liftButtons.keySet()) {
            button.readValue();
            liftButtonVals.put(button, button.wasJustReleased());
        }
    }
    private boolean wasDoingStuff = false;
    @Override
    public void update() {
        if(doingstuff) {
            super.update();
        } else {
            if (wasDoingStuff != doingstuff) liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wasDoingStuff = doingstuff;
            runBoundedTool(liftMotor, Position.MAX.liftPos, gamepad.getLeftY(), false, liftZeroPower);
            runBoundedTool(armMotor, Position.MAX.armPos, -gamepad.getRightY(), false, armZeroPower);
            telemetry.addData("liftpos", liftMotor.getCurrentPosition());
            telemetry.addData("liftMotorPower", liftMotor.getPower());
            telemetry.addData("armpos", armMotor.getCurrentPosition());
            this.rotation.update();
            yReader.readValue();
            if (yReader.getState()) {
                intake.setPower(-1);
                xReader.forceVal(false);
            } else {
                xReader.readValue();
                intake.setPower(xReader.getState() ? 1 : 0);
            }
            bReader.readValue();
            if (bReader.wasJustReleased() && !doingstuff) {
                dump();
            }
            readLiftButtons();
            for (Map.Entry<ButtonReader, Boolean> entry : liftButtonVals.entrySet()) {
                if (entry.getValue() && !doingstuff) {
                    setPosition(Objects.requireNonNull(liftButtons.get(entry.getKey())));
                    doingstuff = true;
                }
            }
        }
    }

    public static void runBoundedTool(@NonNull DcMotor motor, int minBound, int maxBound, double power, boolean negative, double zeroPower) {
        int motorPos = motor.getCurrentPosition() * (negative ? -1 : 1);
        if (((power < 0) && (motorPos > minBound + 4)) || ((power > 0) && (motorPos < maxBound - 4))) {
            motor.setPower(power);
        } else {
            motor.setPower(zeroPower);
        }
    }

    public static void runBoundedTool(@NonNull DcMotor motor, int minBound, int maxBound, double power, boolean negative) {
        runBoundedTool(motor, minBound, maxBound, power, negative, 0);
    }

    public static void runBoundedTool(DcMotor motor, int maxBound, double power, boolean negative, double zeroPower) {
        runBoundedTool(motor, 0, maxBound, power, negative, zeroPower);
    }
    public static void runBoundedTool(DcMotor motor, int maxBound, double power, boolean negative) {
        runBoundedTool(motor, maxBound, power, negative, 0);
    }
}
