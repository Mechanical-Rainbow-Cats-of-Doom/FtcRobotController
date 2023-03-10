package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.BlessedOdo;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.Timer;

@Autonomous
@Config
@Disabled
public class NoRunnerAuto3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BlessedOdo Chassis = new BlessedOdo(hardwareMap, telemetry);
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final AutoTurret turret = new AutoTurret(hardwareMap, 0);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        Thread thread = new Thread(() -> {
            while (opModeIsActive()) {
                tools.update();
            }
        });

        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, true);

        waitForStart();
        thread.start();
        int color = detector.run();
        timer.reset();

        tools.setPosition(AutoTools.Position.NEUTRAL);

        Chassis.path(1000,0,0);
        while(Chassis.Moving() && opModeIsActive()){
            Chassis.drive();
            telemetry.update();
        }


        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
        }
        tools.cleanup();
    }
}
