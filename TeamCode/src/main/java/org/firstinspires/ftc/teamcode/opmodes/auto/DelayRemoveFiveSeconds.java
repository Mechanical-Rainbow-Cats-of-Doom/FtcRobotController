package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;

public class DelayRemoveFiveSeconds extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Current delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        waitForStart();
        DelayStorage.subtractSeconds(5);
        telemetry.addLine("New delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        while (!isStopRequested()) {}
    }
}