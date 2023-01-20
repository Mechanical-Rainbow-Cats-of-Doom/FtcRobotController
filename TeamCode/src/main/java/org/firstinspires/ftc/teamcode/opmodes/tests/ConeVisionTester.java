package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;

@TeleOp
public class ConeVisionTester extends LinearOpMode {
    private static double redMean, blueMean, greenMean;
    public static void setRedMean(double redMean){
        ConeVisionTester.redMean = redMean;
    }
    public static void setGreenMean(double greenMean){
        ConeVisionTester.greenMean = greenMean;
    }
    public static void setBlueMean(double blueMean){
        ConeVisionTester.blueMean = blueMean;
    }
    public static boolean isRed;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, isRed);
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
