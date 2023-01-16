package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import androidx.annotation.NonNull;

@Config
public class TeleOpTools extends AutoTools {
    public static double armZeroPower = 0.15, liftZeroPower = 0.001;
    public double test;
    private final GamepadEx gamepad;
    private final TeleOpTurret turret;
    private final Telemetry telemetry;
    private final ToggleableToggleButtonReader xReader, yReader;
    private final ButtonReader bReader, up, right, down, left_dpad;
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final HashMap<ButtonReader, Position> liftButtons;
    private final HashMap<ButtonReader, Boolean> liftButtonVals = new HashMap<>();
    @Override
    void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public TeleOpTools(HardwareMap hardwareMap, TeleOpTurret turret, GamepadEx toolGamepad, Telemetry telemetry) {
        super(hardwareMap, turret);
        isAuto = false;
        this.turret = turret;
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
    private boolean wasDoingStuff;
    @Override
    public void update() {
        if (wasDoingStuff != doingstuff) liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wasDoingStuff = doingstuff;
        runBoundedTool(liftMotor, Position.MAX.liftPos, gamepad.getLeftY(), false, liftZeroPower);
        telemetry.addData("liftpos", liftMotor.getCurrentPosition());
        double armPower = -gamepad.getRightY();
        runBoundedTool(armMotor, Position.MAX.armPos, armPower, false, armZeroPower);
        telemetry.addData("armpos", armMotor.getCurrentPosition());
        telemetry.addData("armpower", armPower);
        this.turret.update();
        xReader.readValue();
        if (xReader.getState()) {
            intake.setPower(1);
            yReader.forceVal(false);
        } else {
            yReader.readValue();
            intake.setPower(yReader.getState() ? -1 : 0);
        }
        bReader.readValue();
        if (bReader.wasJustReleased() && !doingstuff) {
            dump();
        }
        telemetry.update();
        readLiftButtons();
        for (Map.Entry<ButtonReader, Boolean> entry : liftButtonVals.entrySet()) {
            if (entry.getValue() && !doingstuff) {
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setPosition(Objects.requireNonNull(liftButtons.get(entry.getKey())));
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
