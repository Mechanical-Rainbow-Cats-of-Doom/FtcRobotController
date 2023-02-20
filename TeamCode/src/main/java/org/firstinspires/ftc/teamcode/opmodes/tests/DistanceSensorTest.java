package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous
public class DistanceSensorTest extends LinearOpMode {
    // heck whoever decided that locales are required you are terrible
    static final Locale a = Locale.US;
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor leftSensor = hardwareMap.get(DistanceSensor.class, "leftPoleDetector");
        DistanceSensor rightSensor = hardwareMap.get(DistanceSensor.class, "rightPoleDetector");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("left distance", String.format(a, "%.01f cm", leftSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("right distance", String.format(a, "%.01f cm", rightSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
