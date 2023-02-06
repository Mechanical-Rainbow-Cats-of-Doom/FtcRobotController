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

import org.firstinspires.ftc.teamcode.core.robot.util.AutoStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConePipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Autonomous
public class NoRunnerAuto extends LinearOpMode {
    public int forwardDistance = 33500;
    public int leftDistance = 19857;
    public int rightDistance = -22157;
    public static boolean isRed;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        AutoStorage.waitForDelay(timer);
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
