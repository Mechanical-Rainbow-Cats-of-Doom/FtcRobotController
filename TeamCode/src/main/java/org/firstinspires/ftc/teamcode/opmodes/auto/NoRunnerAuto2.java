package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.AutoStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Timer;

@Autonomous
@Disabled
public class NoRunnerAuto2 extends LinearOpMode {
    public int forwardDistance = 47200;
    public int leftDistance = 8000;
    public int rightDistance = -30200;
    public int leftDistance2 = (-rightDistance)-450;
    public static boolean isRed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final AutoTurret turret = new AutoTurret(hardwareMap, 0);
        turret.setMotorSpeed(0.5);
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
//        tools.waitUntilFinished();
        turret.setPos(0, AutoTurret.Units.DEGREES);
//        tools.waitUntilFinished();
        Thread.sleep(2000);

        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < forwardDistance){
            if(!opModeIsActive()){return;}
            drive.setWeightedDrivePower(new Pose2d(-0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        tools.setIntake(1);
        turret.setPos(-35, AutoTurret.Units.DEGREES);
//        tools.waitUntilFinished();
        Thread.sleep(500);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_NODUMP);
//        tools.waitUntilFinished();
        Thread.sleep(1500);
        FEReset = frontEncoder.getCurrentPosition();
        while (frontEncoder.getCurrentPosition()-FEReset < leftDistance){
            if(!opModeIsActive()){return;}
            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
            telemetry.addData("is it running left", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));

        Thread.sleep(500);
        tools.setIntake(-1);
        Thread.sleep(1000);
        tools.setIntake(0);

        // Cone 5
//        turret.setPos(54, AutoTurret.Units.DEGREES);
//        Thread.sleep(500);
//        tools.setPosition(AutoTools.Position.HOVER_5);
////        tools.waitUntilFinished();
//        Thread.sleep(1000);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > rightDistance){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//
//        tools.setIntake(1);
//        Thread.sleep(1000);
//        tools.setPosition(AutoTools.Position.INTAKE_5);
////        tools.waitUntilFinished();
//        Thread.sleep(1000);
//        tools.setPosition(AutoTools.Position.EXIT_5);
////        tools.waitUntilFinished();
//        Thread.sleep(1500);
//        tools.setIntake(0);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset < Math.round(leftDistance2/2)){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        Thread.sleep(500);
//        tools.setIntake(1);
//        turret.setPos(-35, AutoTurret.Units.DEGREES);
//        Thread.sleep(1000);
//        tools.setPosition(AutoTools.Position.HIGH_TARGET_NODUMP);
////        tools.waitUntilFinished();
//        Thread.sleep(1500);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset < Math.round(leftDistance2/2)){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        Thread.sleep(500);
//        tools.setIntake(-1);
//        Thread.sleep(500);
//        tools.setIntake(0);

        //cone 4
//        turret.setPos(54, AutoTurret.Units.DEGREES);
//        Thread.sleep(500);
//        tools.setPosition(AutoTools.Position.HOVER_4);
//        Thread.sleep(1000);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > rightDistance){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        tools.setIntake(1);
//        tools.setPosition(AutoTools.Position.INTAKE_4);
//        tools.waitUntilFinished(() -> true);
//        tools.setPosition(AutoTools.Position.EXIT_4);
//        tools.waitUntilFinished(() -> true);
//        tools.setIntake(0);

//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > -rightDistance/2){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        tools.setIntake(1);
//        turret.setPos(-35, AutoTurret.Units.DEGREES);
//        tools.setPosition(AutoTools.Position.HIGH_TARGET_NODUMP);
//        tools.waitUntilFinished(() -> {return true;});
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > -rightDistance/2){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        tools.setIntake(-1);
//        Thread.sleep(2000);
//        tools.setIntake(0);

        //cone 3
//        turret.setPos(54, AutoTurret.Units.DEGREES);
//        Thread.sleep(500);
//        tools.setPosition(AutoTools.Position.HOVER_3);
//        Thread.sleep(1000);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > rightDistance){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        tools.setIntake(1);
//        tools.setPosition(AutoTools.Position.INTAKE_3);
//        tools.waitUntilFinished(() -> true);
//        tools.setPosition(AutoTools.Position.EXIT_3);
//        tools.waitUntilFinished(() -> true);
//        tools.setIntake(0);

        // Dump to tile

        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        FEReset = frontEncoder.getCurrentPosition();
        switch (color) {
            case(0):
                turret.setPos(-90, AutoTurret.Units.DEGREES);
//                tools.waitUntilFinished();
                Thread.sleep( 1000);
                tools.setPosition(AutoTools.Position.OFF);
                while (frontEncoder.getCurrentPosition()-FEReset < 8500){
                    if(!opModeIsActive()){return;}
                    drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                break;

            case(1):
                turret.setPos(0, AutoTurret.Units.DEGREES);
//                tools.waitUntilFinished();
                Thread.sleep( 1000);
                tools.setPosition(AutoTools.Position.OFF);
                while (frontEncoder.getCurrentPosition()-FEReset > -10800){
                    if(!opModeIsActive()){return;}
                    drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                break;

            case(2):
                turret.setPos(0, AutoTurret.Units.DEGREES);
//                tools.waitUntilFinished();
                Thread.sleep( 1000);
                tools.setPosition(AutoTools.Position.OFF);
                while (frontEncoder.getCurrentPosition()-FEReset > -33100){
                    if(!opModeIsActive()){return;}
                    drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                break;
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));

        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 > -1000){
            if(!opModeIsActive()){return;}
            drive.setWeightedDrivePower(new Pose2d(0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));


        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
        }
        tools.cleanup();
    }
}
