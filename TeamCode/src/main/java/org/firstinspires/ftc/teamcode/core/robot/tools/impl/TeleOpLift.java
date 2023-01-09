package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.MyToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

public class TeleOpLift extends AutoLift{
    private final GamepadEx gamepad;
    private final TeleOpTurret turret;
    private final Telemetry telemetry;
    @Override
    void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public TeleOpLift(HardwareMap hardwareMap, TeleOpTurret turret, GamepadEx toolGamepad, Telemetry telemetry) {
        super(hardwareMap, turret);
        this.turret = turret;
        gamepad = toolGamepad;
        this.telemetry = telemetry;
    }
    
    @Override
    public void update() {
        runBoundedTool(liftMotor, Position.MAX.liftPos, gamepad.getLeftY(), false, 0.08);
        telemetry.addData("liftpos", liftMotor.getCurrentPosition());
        runBoundedTool(armMotor, Position.MAX.armPos, -gamepad.getRightY(), false, 0.08);
        intake.setPower(gamepad.getButton(GamepadKeys.Button.X) ? -1 : 1);
        //this.turret.update();
        telemetry.update();
    }

    public static void runBoundedTool(@NonNull DcMotor motor, int minBound, int maxBound, double power, boolean negative, double zeroPower) {
        int motorPos = motor.getCurrentPosition() * (negative ? -1 : 1);
        if (((power < 0) && (motorPos > minBound + 4)) || ((power > 0) && (motorPos < maxBound - 4))) {
            motor.setPower(power);
        } else {
            motor.setPower(0);
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
