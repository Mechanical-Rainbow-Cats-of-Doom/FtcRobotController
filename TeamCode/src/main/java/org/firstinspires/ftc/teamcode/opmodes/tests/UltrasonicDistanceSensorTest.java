package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.distance.SEN0304DistanceSensor;

@TeleOp
public class UltrasonicDistanceSensorTest extends LinearOpMode{
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
