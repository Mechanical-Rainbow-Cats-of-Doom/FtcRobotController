package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Timer;
import java.util.function.Consumer;

@Config
public class ControllerTools extends AutoTools {
    // lift and arm zero power moved to auto tools
    public double test;
    private final GamepadEx gamepad;
    private final ControllerTurret turret;
    private final Telemetry telemetry;
    private final ToggleableToggleButtonReader xReader, yReader;
    private final ButtonReader bReader;
    private final LinearOpMode opMode;
    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final LinkedHashMap<ButtonReader, Position> liftButtons;
    private final HashMap<ButtonReader, Boolean> liftButtonVals = new HashMap<>();

    private final BoxedBoolean[] wasOn = {new BoxedBoolean(), new BoxedBoolean()};

    static class BoxedBoolean {
        boolean value = false;
    }

    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public ControllerTools(HardwareMap hardwareMap, Timer timer, GamepadEx toolGamepad, GamepadEx driveGamepad, Telemetry telemetry, LinearOpMode opMode) {
        super(hardwareMap, timer, null);
        isAuto = false;
        gamepad = toolGamepad;
        this.turret = new ControllerTurret(hardwareMap, driveGamepad);
        super.turret = turret;
        this.telemetry = telemetry;
        this.opMode = opMode;
        this.xReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.X);
        this.yReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.Y);
        this.bReader = new ButtonReader(gamepad, GamepadKeys.Button.B);
        this.liftButtons = new LinkedHashMap<ButtonReader, Position>() {{
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP), Position.HIGH_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN), Position.GROUND_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT), Position.LOW_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT), Position.MEDIUM_TARGET_NODUMP);
        }};
    }

    public void initIntake() {
        final Thread thread = new Thread(() -> {
            int oldPower = 0;
            while (!opMode.isStopRequested()) {
                yReader.readValue();
                if (yReader.getState()) {
                    if (oldPower != -1) {
                        intake.setPower(-1);
                        xReader.forceVal(false);
                        oldPower = -1;
                    }
                } else {
                    int newPower = xReader.getState() ? 1 : 0;
                    if (newPower != oldPower) {
                        intake.setPower(newPower);
                        oldPower = newPower;
                    }
                }
            }
        });
        thread.setPriority(Thread.MAX_PRIORITY-2);
        thread.start();
    }

    public static void readLiftButtons(@NonNull Map<ButtonReader, Boolean> buttonVals, @NonNull Map<ButtonReader, ?> buttons) {
        for (ButtonReader button : buttons.keySet()) {
            button.readValue();
            buttonVals.put(button, button.wasJustReleased());
        }
    }

    private boolean wasDoingStuff = false;
    private void cleanupOpMode() {
        doingstuff = false;
        armMotor.setPower(0);
        liftMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static <Pos> boolean setPosFromButtonMap(Map<ButtonReader, Boolean> buttonVals, Map<ButtonReader, Pos> buttonMap, Consumer<Pos> consumer) {
        readLiftButtons(buttonVals, buttonMap);
        for (Map.Entry<ButtonReader, Boolean> entry : buttonVals.entrySet()) {
            if (entry.getValue()) {
                consumer.accept(Objects.requireNonNull(buttonMap.get(entry.getKey())));
                return true;
            }
        }
        return false;
    }

    @Override
    public void update() {
        final double right = gamepad.getRightY();
        final double left = gamepad.getLeftY();

        doingstuff = setPosFromButtonMap(liftButtonVals, liftButtons, this::setPosition);

        if (doingstuff) {
            if (Math.abs(right) > 0.05 || Math.abs(left) > 0.05) {
                cleanupOpMode();
            } else if(bReader.wasJustReleased() && !doingstuff) {
                cleanupOpMode();
                dump();
            } else {
                super.update();
                return;
            }
        }
        if (wasDoingStuff) liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wasDoingStuff = doingstuff;
        runBoundedTool(liftMotor, wasOn[0], Position.MAX.liftPos, left, false, liftZeroPower);
        runBoundedTool(armMotor, wasOn[1], Position.MAX.armPos, -right, false, armZeroPower);
        telemetry.addData("liftpos", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotorPower", liftMotor.getPower());
        telemetry.addData("armpos", armMotor.getCurrentPosition());
        turret.update();
        bReader.readValue();
    }


    public static void runBoundedTool(@NonNull DcMotor motor, BoxedBoolean previouslyOn, int minBound, int maxBound, double power, boolean negative, double zeroPower) {
        int motorPos = motor.getCurrentPosition() * (negative ? -1 : 1);

        if (((power < 0) && (motorPos > minBound + 4)) || ((power > 0) && (motorPos < maxBound - 4))) {
            motor.setPower(power);
            previouslyOn.value = true;
        } else {
            if(previouslyOn.value) {
                motor.setPower(zeroPower);
                previouslyOn.value = false;
            }
        }
    }

    public static void runBoundedTool(@NonNull DcMotor motor, BoxedBoolean previouslyOn, int minBound, int maxBound, double power, boolean negative) {
        runBoundedTool(motor, previouslyOn, minBound, maxBound, power, negative, 0);
    }

    public static void runBoundedTool(DcMotor motor, BoxedBoolean previouslyOn, int maxBound, double power, boolean negative, double zeroPower) {
        runBoundedTool(motor, previouslyOn, 0, maxBound, power, negative, zeroPower);
    }
    public static void runBoundedTool(DcMotor motor, BoxedBoolean previouslyOn, int maxBound, double power, boolean negative) {
        runBoundedTool(motor, previouslyOn, maxBound, power, negative, 0);
    }
}
