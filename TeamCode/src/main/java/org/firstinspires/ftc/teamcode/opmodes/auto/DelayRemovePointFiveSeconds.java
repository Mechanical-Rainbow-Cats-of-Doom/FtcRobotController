package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.util.DelayStorage;

@Autonomous
public class DelayRemovePointFiveSeconds extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Current seconds: " + DelayStorage.seconds);
        telemetry.update();
        waitForStart();
        DelayStorage.subtractSeconds(0.5);
        telemetry.addLine("Current seconds: " + DelayStorage.seconds);
        telemetry.update();
        while (!isStopRequested()) {}
    }
}