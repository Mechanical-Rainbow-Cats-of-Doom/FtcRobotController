package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous
public class DistanceSensorTestButForStraightPeople extends LinearOpMode {
    // heck whoever decided that locales are required you are terrible
    static final Locale a = Locale.US;
    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        Rev2mDistanceSensorEx leftSensor = hardwareMap.get(Rev2mDistanceSensorEx.class, "leftPoleDetector");
        Rev2mDistanceSensorEx rightSensor = hardwareMap.get(Rev2mDistanceSensorEx.class, "rightPoleDetector");
        leftSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);
        rightSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("left distance", String.format(a, "%.01f cm", leftSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("right distance", String.format(a, "%.01f cm", rightSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
