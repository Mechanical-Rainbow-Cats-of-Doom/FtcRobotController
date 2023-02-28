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
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.Cycler;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Timer;
import java.util.function.Consumer;

@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
@Config
public class ControllerTools extends AutoTools {
    // lift and arm zero power moved to auto tools
    private final GamepadEx gamepad;
    private final ControllerTurret turret;
    private final Telemetry telemetry;
    private final ToggleableToggleButtonReader xReader, yReader, toolCapHeight;
    private final ButtonReader backReader, leftStickClickReader;
    private final LinkedHashMap<ButtonReader, Position> liftButtons;
    private final LinkedHashMap<ButtonReader, Cycler.Cycles> cycleButtons;
    private final HashMap<ButtonReader, Boolean> liftButtonVals = new HashMap<>(), cycleButtonVals = new HashMap<>();
    private final BoxedBoolean[] wasOn = {new BoxedBoolean(), new BoxedBoolean()};
    private double oldIntakePower = 0.0;

    public static class BoxedBoolean {
        public boolean value;
        public BoxedBoolean(boolean value) {
            this.value = value;
        }
        public BoxedBoolean() {
            this(false);
        }
    }

    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public ControllerTools(HardwareMap hardwareMap, Timer timer, GamepadEx toolGamepad, GamepadEx driveGamepad, Telemetry telemetry, LinearOpMode opMode) {
        super(hardwareMap, timer, null, opMode, telemetry);
        isAuto = false;
        gamepad = toolGamepad;
        this.turret = new ControllerTurret(hardwareMap, driveGamepad, toolGamepad, 0); // fin fix this plz thx
        super.turret = turret;
        this.telemetry = telemetry;
        this.xReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.X);
        this.yReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.Y);
        this.backReader = new ButtonReader(driveGamepad, GamepadKeys.Button.BACK);
        this.leftStickClickReader = new ButtonReader(driveGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        this.toolCapHeight = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.START, true);
        this.liftButtons = new LinkedHashMap<ButtonReader, Position>() {{
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP), Position.HIGH_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN), Position.GROUND_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT), Position.LOW_TARGET_NODUMP);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT), Position.MEDIUM_TARGET_NODUMP);
        }};
        this.cycleButtons = new LinkedHashMap<ButtonReader, Cycler.Cycles>() {{
           put(new ButtonReader(driveGamepad, GamepadKeys.Button.X), Cycler.Cycles.ALLIANCE_LEFT_HIGH);
           put(new ButtonReader(driveGamepad, GamepadKeys.Button.B), Cycler.Cycles.ALLIANCE_RIGHT_HIGH);
           put(new ButtonReader(driveGamepad, GamepadKeys.Button.Y), Cycler.Cycles.STACK_LEFT_HIGH);
           put(new ButtonReader(driveGamepad, GamepadKeys.Button.A), Cycler.Cycles.STACK_RIGHT_HIGH);
        }};
    }

    private void cleanupOpMode() {
        doingstuff.value = false;
        armMotor.setPower(armZeroPower);
        liftMotor.setPower(liftZeroPower);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        needsToChangeMode = true;
    }

    @Override
    public void stopCycling() {
        super.stopCycling();
        cleanupOpMode();
    }

    public static void readButtons(@NonNull Map<ButtonReader, Boolean> buttonVals, @NonNull Map<ButtonReader, ?> buttons) {
        for (ButtonReader button : buttons.keySet()) {
            button.readValue();
            buttonVals.put(button, button.wasJustReleased());
        }
    }

    public static <Pos> void setPosFromButtonMap(Map<ButtonReader, Boolean> buttonVals, Map<ButtonReader, Pos> buttonMap, BoxedBoolean doingStuff, Consumer<Pos> consumer) {
        readButtons(buttonVals, buttonMap);
        for (Map.Entry<ButtonReader, Boolean> entry : buttonVals.entrySet()) {
            if (entry.getValue()) {
                consumer.accept(Objects.requireNonNull(buttonMap.get(entry.getKey())));
                doingStuff.value = true;
                return;
            }
        }
    }

    @Override
    public void update() {
        telemetry.addData("liftpos", liftMotor.getCurrentPosition());
        telemetry.addData("liftMotorPower", liftMotor.getPower());
        telemetry.addData("armpos", armMotor.getCurrentPosition());
        if (cycling) {
            leftStickClickReader.readValue();
            if (leftStickClickReader.wasJustReleased()) endCyclingEarly(false);
            else {
                super.update();
                return;
            }
        }
        setPosFromButtonMap(cycleButtonVals, cycleButtons, doingstuff, (cycleType) ->
            startCycling(cycleType, () -> {
                backReader.readValue();
                return backReader.wasJustReleased();
            }, false)
        );
        if (cycling) {
            super.update();
            return;
        }
        turret.whopper();
        yReader.readValue();
        if (yReader.getState()) {
            if (oldIntakePower != 0) {
                // this sucks but apparently is faster but it probably wont have any affect on the
                // performance because the underlying code is shit
                // also it will make ethan mad and i can call this ++i v2 (except it changes the code)
                intake.getController().setServoPosition(intake.getPortNumber(), 0);
                xReader.forceVal(false);
                oldIntakePower = 0;
            }
        } else {
            xReader.readValue();
            double newPower = xReader.getState() ? 1 : 0.5;
            if (newPower != oldIntakePower) {
                intake.getController().setServoPosition(intake.getPortNumber(), newPower);
                oldIntakePower = newPower;
            }
        }

        final double right = gamepad.getRightY();
        final double left = gamepad.getLeftY();
        setPosFromButtonMap(liftButtonVals, liftButtons, doingstuff, this::setPosition);

        if (doingstuff.value) {
            if (Math.abs(right) > 0.05 || Math.abs(left) > 0.05) {
                cleanupOpMode();
            } else {
                super.update();
                return;
            }
        }
        toolCapHeight.readValue();
        runBoundedTool(liftMotor, wasOn[0], toolCapHeight.getState() ? Position.MAX.liftPos : Integer.MAX_VALUE, left, false, liftZeroPower);
        runBoundedTool(armMotor, wasOn[1], toolCapHeight.getState() ? Position.MAX.armPos : Integer.MAX_VALUE, -right, false, armZeroPower);
        if (cyclerArm.isBusy()) cyclerArm.debugUpdate();
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
