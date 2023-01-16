package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

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

import androidx.annotation.NonNull;

@Config
public class TeleOpTools extends AutoTools {
    public static double armZeroPower = 0, liftZeroPower = 0;
    public double test;
    private final GamepadEx gamepad;
    private final TeleOpTurret turret;
    private final Telemetry telemetry;
    private final ToggleableToggleButtonReader xReader, yReader;
    private final ButtonReader bReader;
    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public TeleOpTools(HardwareMap hardwareMap, TeleOpTurret turret, GamepadEx toolGamepad, Telemetry telemetry) {
        super(hardwareMap, turret);
        this.turret = turret;
        gamepad = toolGamepad;
        this.telemetry = telemetry;
        this.xReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.X);
        this.yReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.Y);
        this.bReader = new ButtonReader(gamepad, GamepadKeys.Button.B);
    }


    @Override
    public void update() {
        runBoundedTool(liftMotor, Position.MAX.liftPos, gamepad.getLeftY(), false, liftZeroPower);
        telemetry.addData("liftpos", liftMotor.getCurrentPosition());
        double armPower = -gamepad.getRightY();
        armMotor.setPower(armPower < 0 ? Math.min(armPower, -armZeroPower) : Math.max(armPower, armZeroPower));
        telemetry.addData("armpos", armMotor.getCurrentPosition());
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
        if (bReader.wasJustReleased() && !dumping) {
            dump(false);
        }
        telemetry.update();
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
