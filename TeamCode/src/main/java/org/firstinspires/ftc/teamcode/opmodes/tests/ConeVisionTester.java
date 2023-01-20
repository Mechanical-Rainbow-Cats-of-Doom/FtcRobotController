package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;

@TeleOp
public class ConeVisionTester extends LinearOpMode {
    public static double redMean, blueMean, greenMean;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, false, telemetry);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("rgb", detector.run());
            telemetry.addData("red", redMean);
            telemetry.addData("green", greenMean);
            telemetry.addData("blue", blueMean);
            telemetry.update();
        }
    }
}
