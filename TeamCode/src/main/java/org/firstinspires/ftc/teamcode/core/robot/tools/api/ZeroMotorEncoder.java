package org.firstinspires.ftc.teamcode.core.robot.tools.api;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import androidx.annotation.NonNull;

public class ZeroMotorEncoder {
    public static void zero(@NonNull DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

    }
}
