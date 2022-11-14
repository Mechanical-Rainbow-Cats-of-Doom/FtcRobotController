package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.util.DelayStorage;

@Autonomous
public class DelayAddOneSecond extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Current delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        waitForStart();
        DelayStorage.addSeconds(1);
        telemetry.addLine("New delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        while (!isStopRequested()) {}
    }
}
