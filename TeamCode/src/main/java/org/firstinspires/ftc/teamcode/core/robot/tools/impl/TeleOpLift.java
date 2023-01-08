package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.MyToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

public class TeleOpLift extends AutoLift{
    private final GamepadEx gamepad;
    private final MyToggleButtonReader xReader;
    private final TeleOpTurret turret;
    
    @Override
    void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_USING_ENCODER);
        ZeroMotorEncoder.zero(armMotor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public TeleOpLift(HardwareMap hardwareMap, TeleOpTurret turret, GamepadEx toolGamepad) {
        super(hardwareMap, turret);
        this.turret = turret;
        gamepad = toolGamepad;
        xReader = new MyToggleButtonReader(gamepad, GamepadKeys.Button.X); // this button reader kind of sus yo might not work
    }
    
    @Override
    public void update() {
        runBoundedTool(liftMotor, Position.MAX.motorPos, gamepad.getLeftY());
        runBoundedTool(armMotor, Position.MAX.armPos, gamepad.getRightY());
        intake.setPower(xReader.update() ? 1 : -1);
        this.turret.update();
    }
    
    public static void runBoundedTool(@NonNull DcMotor motor, int minBound, int maxBound, double power) {
        int motorPos = motor.getCurrentPosition();
        if (((power < 0) && (motorPos > minBound + 4)) || ((power > 0) && (motorPos < maxBound - 4))) {
            motor.setPower(power);
        } else {
            motor.setPower(0);
        }
    }
    
    public static void runBoundedTool(DcMotor motor, int maxBound, double power) {
        runBoundedTool(motor, 0, maxBound, power);
    }
}
