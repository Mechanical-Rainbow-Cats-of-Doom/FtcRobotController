package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.core.robot.distance.SEN0304DistanceSensor;

import java.util.ArrayList;

@TeleOp
public class DisatnceSensorTest extends LinearOpMode{
    SEN0304DistanceSensor distanceSensor = new SEN0304DistanceSensor(hardwareMap.get(I2cDeviceSynch.class, "frontSensor"));
    int distanceResult = 0;
    ArrayList<Integer> distanceResults = new ArrayList<Integer>();
    final MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    Thread DSthread = new Thread(() -> {
        this.distanceSensor.readDistance();
        try {
            Thread.sleep(20);
        }  catch (InterruptedException e) {
            e.printStackTrace();
        }
        distanceResults.add(this.distanceSensor.getDistance());
    });
    @Override
    public void runOpMode() {
        telemetry.addData("Distance: ", distanceResult);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.x) {
                DSthread.start();
                while (gamepad1.x) {
                }
            }
            if (gamepad1.y) {
                for (int i = 0; i < distanceResults.size(); i++) {
                    distanceResult += distanceResults.indexOf(i);
                    if (i == distanceResults.size()) {
                        distanceResult /= distanceResults.size();
                    }
                }
                while (gamepad1.y) {
                }
            }

            telemetry.update();
        }
    }
}
