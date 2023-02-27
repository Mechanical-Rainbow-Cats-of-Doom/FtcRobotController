package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.tools.BetterDistanceSensor;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.Cycler;

@TeleOp
@Config
public class LaserDistanceSensorTest extends LinearOpMode{
    public static double requestRate = 50;
    public static DistanceUnit unit = DistanceUnit.CM;
    @Override
    public void runOpMode() throws InterruptedException {
        final BetterDistanceSensor distanceSensor = new BetterDistanceSensor(hardwareMap, "distanceSensor", requestRate, unit);
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        waitForStart();
        while(opModeIsActive()) {
            final double output = distanceSensor.request();
            telemetry.addData("Distance", output);
            telemetry.addData("Is Detected", output < Cycler.isObjectDistance);
            telemetry.update();
        }
    }
}
