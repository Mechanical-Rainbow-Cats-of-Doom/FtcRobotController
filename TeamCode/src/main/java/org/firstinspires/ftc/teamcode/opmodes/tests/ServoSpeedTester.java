package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Disabled
public class ServoSpeedTester extends LinearOpMode {
    @Config
    public static class ServoSpeed {
        public static double SPEED = 0.1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "intakeServo");
        waitForStart();
        while(!isStopRequested()) {
            servo.setPower(ServoSpeed.SPEED);
        }
    }
}
