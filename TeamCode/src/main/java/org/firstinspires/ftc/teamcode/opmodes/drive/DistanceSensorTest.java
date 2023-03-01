package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.distance.MultipleDistanceSensors;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MultipleDistanceSensors sensors = new MultipleDistanceSensors(hardwareMap, this);
        waitForStart();
        sensors.init(); // front right back left
        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                telemetry.addData("distance " + i, sensors.getDistance(i));
                telemetry.addData("isObject " + i, sensors.isObject(i, 5));
            }
            telemetry.update();
        }
    }
}
