package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;

@TeleOp
public class ConeVisionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, false, telemetry);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("cmy: ", detector.run());
            telemetry.update();
        }
    }
}
