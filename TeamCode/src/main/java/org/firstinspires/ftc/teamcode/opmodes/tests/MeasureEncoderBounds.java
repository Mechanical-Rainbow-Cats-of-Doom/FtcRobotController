package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.util.BetterButtonReader;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

@Config
@TeleOp
@Disabled
public class MeasureEncoderBounds extends LinearOpMode {
    public static String motorName = "turret";
    public static int loopCount = 5;

    public static void waitForButtonReader(@NonNull BetterButtonReader reader) {
        while (!reader.wasJustReleased()) {
            reader.readValue();
        }
    }

    @Override
    public void runOpMode() {
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        final DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        final BetterButtonReader aReader = new BetterButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.A);
        telemetry.addLine("Motor zeroed on start");
        telemetry.update();
        waitForStart();
        final int[] motorVals = new int[loopCount];
        for (int i = 1; i <= loopCount; i++) {
            ZeroMotorEncoder.zero(motor, DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addLine("Loop count " + i + "of " + loopCount);
            telemetry.addLine("Press A once lift is at top");
            telemetry.update();
            waitForButtonReader(aReader);
            motorVals[i-1] = motor.getCurrentPosition();
            telemetry.addData("Encoder", motorVals[i-1]);
            telemetry.addLine("Press A once lift is at bottom");
            telemetry.update();
            waitForButtonReader(aReader);

        }
        int sum = 0;
        for (int val : motorVals) sum += val;
        telemetry.addData("Average of all encoder values is", sum/(double) loopCount);
        telemetry.addLine("Press A to terminate.");
        telemetry.update();
        waitForButtonReader(aReader);
        requestOpModeStop();
    }

}
