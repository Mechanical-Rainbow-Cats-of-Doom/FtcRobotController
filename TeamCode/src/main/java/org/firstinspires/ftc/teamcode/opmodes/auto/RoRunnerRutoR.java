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
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Timer;

@Autonomous
public class RoRunnerRutoR extends LinearOpMode {
    public int forwardDistance = 46200;
    public int placeCone1 = -8000;
    public int pickUpCone5 = 30200;
    public int rightDistance2 = (-pickUpCone5)+450;
    public static boolean isRed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean E_Stop = false;
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final AutoTurret turret = new AutoTurret(hardwareMap);
        turret.setMotorSpeed(0.5);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this);
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

        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        timer.reset();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < forwardDistance){
            if(!opModeIsActive()){return;}
            if(timer.seconds() > 7){
                E_Stop = true;
                break;
            }
            drive.setWeightedDrivePower(new Pose2d(-0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        if(E_Stop){
            tools.setPosition(AutoTools.Position.OFF);
            while(opModeIsActive()){
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
            }
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        tools.setIntake(1);
        turret.setPos(-35, AutoTurret.Units.DEGREES);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_NODUMP);
        Thread.sleep(2000);
        FEReset = frontEncoder.getCurrentPosition();
        timer.reset();
        while (frontEncoder.getCurrentPosition()-FEReset > placeCone1){
            if(!opModeIsActive()){return;}
            if(timer.seconds() > 7){
                E_Stop = true;
                break;
            }
            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
            telemetry.addData("is it running left", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        if(E_Stop){
            tools.setPosition(AutoTools.Position.OFF);
            while(opModeIsActive()){
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
            }
        }

        Thread.sleep(500);
        tools.setIntake(-1);
        Thread.sleep(1000);
        tools.setIntake(0);

        // Cone 5
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//        turret.setPos(-126, AutoTurret.Units.DEGREES);
//        Thread.sleep(500);
//        tools.setPosition(AutoTools.Position.HOVER_5);
//        Thread.sleep(1000);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset < pickUpCone5){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//
//
//        tools.setIntake(1);
//        Thread.sleep(1000);
//        tools.setPosition(AutoTools.Position.INTAKE_5);
//        Thread.sleep(2000);
//        tools.setPosition(AutoTools.Position.EXIT_5);
//        Thread.sleep(500);
//        tools.setIntake(0);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > Math.round(rightDistance2 /2)){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
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
//        Thread.sleep(500);
//
//        REReset = -rightEncoder.getCurrentPosition();
//        LEReset = leftEncoder.getCurrentPosition();
//        FEReset = frontEncoder.getCurrentPosition();
//        while (frontEncoder.getCurrentPosition()-FEReset > Math.round(rightDistance2 /2)){
//            if(!opModeIsActive()){return;}
//            drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
//            telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
//            telemetry.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//
//        Thread.sleep(1000);
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
            case(2):
                turret.setPos(145, AutoTurret.Units.DEGREES);
                Thread.sleep( 500);
                tools.setPosition(AutoTools.Position.OFF);
                timer.reset();
                while (frontEncoder.getCurrentPosition()-FEReset > -8500){
                    if(!opModeIsActive()){return;}
                    if(timer.seconds() > 7){
                        E_Stop = true;
                        break;
                    }
                    drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                if(E_Stop){
                    tools.setPosition(AutoTools.Position.OFF);
                    while(opModeIsActive()){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                    }
                }
                break;

            case(1):
                turret.setPos(145, AutoTurret.Units.DEGREES);
                Thread.sleep( 500);
                tools.setPosition(AutoTools.Position.OFF);
                timer.reset();
                while (frontEncoder.getCurrentPosition()-FEReset < 10800){
                    if(!opModeIsActive()){return;}
                    if(timer.seconds() > 7){
                        E_Stop = true;
                        break;
                    }
                    drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                if(E_Stop){
                    tools.setPosition(AutoTools.Position.OFF);
                    while(opModeIsActive()){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                    }
                }
                break;

            case(0):
                turret.setPos(145, AutoTurret.Units.DEGREES);
                Thread.sleep( 500);
                tools.setPosition(AutoTools.Position.OFF);
                timer.reset();
                while (frontEncoder.getCurrentPosition()-FEReset < 30100){
                    if(!opModeIsActive()){return;}
                    if(timer.seconds() > 7){
                        E_Stop = true;
                        break;
                    }
                    drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
                    telemetry.addData("is it running right", frontEncoder.getCurrentPosition());
                    telemetry.update();
                }
                if(E_Stop){
                    tools.setPosition(AutoTools.Position.OFF);
                    while(opModeIsActive()){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                    }
                }
                break;
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));

        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        timer.reset();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 > -1000){
            if(!opModeIsActive()){return;}
            if(timer.seconds() > 7){
                E_Stop = true;
                break;
            }
            drive.setWeightedDrivePower(new Pose2d(0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        if(E_Stop){
            tools.setPosition(AutoTools.Position.OFF);
            while(opModeIsActive()){
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
            }
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));


        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(0,0,0));
        }
        tools.cleanup();
    }
}
