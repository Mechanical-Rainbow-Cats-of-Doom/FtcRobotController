package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.AutoStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.BlessedOdo;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Timer;

@Autonomous
public class NoRunnerAuto3 extends LinearOpMode {
    public int forwardDistance = -47000;
    public int leftDistance = 8000;
    public int rightDistance = -27848;
    public static boolean isRed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BlessedOdo Chassis = new BlessedOdo(hardwareMap);
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final AutoTurret turret = new AutoTurret(hardwareMap);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        Thread thread = new Thread(() -> {
            while (opModeIsActive()) {
                tools.update();
            }
        });

        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        int LEReset;
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        int REReset;
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        int FEReset;
        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, isRed);

        waitForStart();
        thread.start();
        int color = detector.run();
        timer.reset();
        telemetry.addData("is it running delay", Math.random());
        telemetry.update();
        AutoStorage.waitForDelay(timer);

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        turret.setPos(0, AutoTurret.Units.DEGREES);

        Chassis.Path(forwardDistance,0,0);
        while(Chassis.Moving() && opModeIsActive()){
            Chassis.Drive();
        }


        tools.setIntake(1);
        turret.setPos(-35, AutoTurret.Units.DEGREES);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_NODUMP);
        Thread.sleep(2000);

        Chassis.Path(0,leftDistance,0);
        while(Chassis.Moving() && opModeIsActive()){
            Chassis.Drive();
        }

        tools.setIntake(-1);
        Thread.sleep(2000);
        tools.setIntake(0);
        turret.setPos(80, AutoTurret.Units.DEGREES);
        Thread.sleep(500);
        tools.setPosition(AutoTools.Position.HOVER_5);
        Thread.sleep(1000);

        Chassis.Path(0,rightDistance,0);
        while(Chassis.Moving() && opModeIsActive()){
            Chassis.Drive();
        }

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
        }
        tools.cleanup();
    }
}
