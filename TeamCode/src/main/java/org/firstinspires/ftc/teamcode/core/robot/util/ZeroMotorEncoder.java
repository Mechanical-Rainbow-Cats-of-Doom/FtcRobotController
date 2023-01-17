package org.firstinspires.ftc.teamcode.core.robot.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import androidx.annotation.NonNull;

public class ZeroMotorEncoder {
    /**
     * Blocks for >100ms
     * @param endRunMode if this is set to {@link DcMotor.RunMode#RUN_TO_POSITION} your motor power will be set to 1, deal with it
     */
    public static void zero(@NonNull DcMotor motor, DcMotor.RunMode endRunMode) {
        if (endRunMode == DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setPower(0);
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(endRunMode);
        if (endRunMode == DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setPower(1);
        }
    }

    /**
     * default is just running to position see other javadoc
     */
    public static void zero(@NonNull DcMotor motor) {
        zero(motor, DcMotor.RunMode.RUN_TO_POSITION);
    }
}
