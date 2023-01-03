package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;

@Autonomous
public class DelayAddPointFiveSeconds extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Current delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        waitForStart();
        DelayStorage.addSeconds(0.5);
        telemetry.addLine("New delay: " + DelayStorage.seconds + " seconds");
        telemetry.update();
        while (!isStopRequested()) {}
    }
}
