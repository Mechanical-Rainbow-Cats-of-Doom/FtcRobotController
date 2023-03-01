package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class ServoTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "intake");
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        double[] elapsedStarting = new double[3];
        double[] elapsedEnding = new double[3];
        for (int i = 0; i < 3; i++) {
            timer.reset();
            servo.setPower(1);
            while(servo.getPower() != 1);
            elapsedStarting[i] = timer.milliseconds();

            // update telemetry
            for (int j = 0; j < i; j++) {
                telemetry.addData("Run " + j + " starting", elapsedStarting[j]);
                if (j < i-1) {
                    telemetry.addData("Run " + (j) + " ending", elapsedEnding[j]);
                }
            }
            telemetry.update();

            timer.reset();
            servo.setPower(0);
            elapsedEnding[i] = timer.milliseconds();

            // update telemetry
            for (int j = 0; j < i; j++) {
                telemetry.addData("Run " + j + " starting", elapsedStarting[j]);
                telemetry.addData("Run " + j + " ending", elapsedEnding[j]);
            }
        }
        while(!isStopRequested());
    }
}
