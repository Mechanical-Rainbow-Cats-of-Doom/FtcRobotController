package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class GetEncoderTicksFromMotorName extends LinearOpMode {
    public static String encoderName = "";
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor dcMotor = hardwareMap.get(DcMotor.class, encoderName);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("cur pos", dcMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
