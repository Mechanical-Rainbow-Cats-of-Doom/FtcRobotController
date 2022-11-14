package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.teamcode.core.robot.distance.SEN0304DistanceSensor;

import java.util.ArrayList;

@TeleOp
public class DisatnceSensorTest extends LinearOpMode{
    SEN0304DistanceSensor distanceSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(SEN0304DistanceSensor.class, "frontSensor");

        waitForStart();
        while(opModeIsActive()) {
            distanceSensor.readDistance();
            Thread.sleep(20);
            telemetry.addData("Distance", distanceSensor.getDistance());
            telemetry.update();
        }
    }
}
