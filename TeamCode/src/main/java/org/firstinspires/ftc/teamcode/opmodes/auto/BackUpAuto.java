package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConePipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Timer;

@Autonomous
public class BackUpAuto extends LinearOpMode {
    public int forwardDistance = 33500;
    public int leftDistance = 19857;
    public int rightDistance = -22157;
    public static boolean isRed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final AutoTurret turret = new AutoTurret(hardwareMap);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this);
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        int LEReset;
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        int REReset;
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        int FEReset;
        final ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, isRed);
        final Thread thread = new Thread(() -> {
            final GamepadEx moveGamepad = new GamepadEx(gamepad1);
            final ButtonReader up = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_UP);
            final ButtonReader left = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_LEFT);
            final ButtonReader down = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_DOWN);
            final ButtonReader right = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_RIGHT);
            while (!isStarted()) {
                up.readValue();
                left.readValue();
                down.readValue();
                right.readValue();
                if (up.wasJustReleased()) ConePipeline.topRectHeightPercentage -= 0.01;
                if (down.wasJustReleased()) ConePipeline.topRectHeightPercentage += 0.01;
                if (left.wasJustReleased()) ConePipeline.topRectWidthPercentage -= 0.01;
                if (right.wasJustReleased()) ConePipeline.topRectWidthPercentage += 0.01;
            }
        });
        thread.start();
        waitForStart();
        thread.interrupt();
        int color = detector.run();
        timer.reset();
        telemetry.addData("is it running delay", Math.random());
        telemetry.update();
        DelayStorage.waitForDelay(timer);
        tools.setPosition(AutoTools.Position.NEUTRAL);
        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < Math.round(forwardDistance*1.5)){
            if(!opModeIsActive()){return;}
            drive.setWeightedDrivePower(new Pose2d(-0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        tools.setPosition(AutoTools.Position.MEDIUM_TARGET_NODUMP);
        Pose2d endPose = drive.getPoseEstimate();
        while (!isStopRequested() || timer.seconds()<20) {
            StayInPosition.stayInPose(drive, endPose);
        }
        tools.setPosition(AutoTools.Position.NEUTRAL);
        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        double stuckTime = timer.seconds();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < Math.round(forwardDistance*-1.5)){
            if(!opModeIsActive()){return;}
            if((timer.seconds() - stuckTime) > 5 && ((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < 1000){
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                telemetry.addLine("Paused cause' stuck 😇😇😇");
                telemetry.update();
            }
            drive.setWeightedDrivePower(new Pose2d(0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        FEReset = frontEncoder.getCurrentPosition();
        switch (color){
            case 0:
                while ((frontEncoder.getCurrentPosition()-FEReset) < leftDistance){
                    if(!opModeIsActive()){return;}
                    drive.setWeightedDrivePower(new Pose2d(0,-0.5,0));
                    telemetry.addData("is it running 0", (frontEncoder.getCurrentPosition()-FEReset));
                    telemetry.update();
                }
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                break;
            case 1:
                telemetry.addData("is it running 1", Math.random());
                telemetry.update();
                break;
            case 2:
                while ((frontEncoder.getCurrentPosition()-FEReset) > rightDistance){
                    if(!opModeIsActive()){return;}
                    drive.setWeightedDrivePower(new Pose2d(0,0.5,0));
                    telemetry.addData("is it running 2", (frontEncoder.getCurrentPosition()-FEReset));
                    telemetry.update();
                }
                drive.setWeightedDrivePower(new Pose2d(0,0,0));
                break;
        }
        REReset = -rightEncoder.getCurrentPosition();
        LEReset = leftEncoder.getCurrentPosition();
        while (((leftEncoder.getCurrentPosition()-LEReset)+(-rightEncoder.getCurrentPosition()-REReset))/2 < forwardDistance){
            if(!opModeIsActive()){return;}
            drive.setWeightedDrivePower(new Pose2d(-0.5,0,0));
            telemetry.addData("is it running drive", ((leftEncoder.getCurrentPosition()-LEReset)+(rightEncoder.getCurrentPosition()-REReset))/2);
            telemetry.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
    }
}
