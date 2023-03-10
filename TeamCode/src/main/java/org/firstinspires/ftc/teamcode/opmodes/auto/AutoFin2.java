package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConePipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoFin extends LinearOpMode {
    protected boolean mirror = false;
    private final ElapsedTime autoTimer = new ElapsedTime();

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        
        final double startingHeading = Math.toRadians(90);
        final Pose2d startPose = cMirrorY(new Pose2d(-32, 63, startingHeading), mirror);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        final Pose2d middlePosition = cMirrorY(new Pose2d(-35, 10, startingHeading), mirror);

        TrajectorySequence start = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(cMirrorY(new Vector2d(-35, 63), mirror))
                .lineToLinearHeading(middlePosition)
                .build();

        final Pose2d dropPosition = cMirrorY(new Pose2d(-22, 10, startingHeading), mirror);

        TrajectorySequence middleToHigh = drive.trajectorySequenceBuilder(middlePosition)
                .lineToLinearHeading(dropPosition)
                .build();

        final Pose2d intakePosition = cMirrorY(new Pose2d(-60, 12, startingHeading), mirror);

        TrajectorySequence intake = drive.trajectorySequenceBuilder(dropPosition)
                .strafeTo(cMirrorY(new Vector2d(-22,13), mirror))
                .strafeTo(cMirrorY(new Vector2d(-57, 13), mirror))
                .lineToLinearHeading(intakePosition)
                .build();

        TrajectorySequence intakeToMiddle = drive.trajectorySequenceBuilder(cMirrorY(new Pose2d(-57, 12, startingHeading), mirror))
                .lineToLinearHeading(middlePosition)
                .build();

        final Pose2d park0Position = cMirrorY(new Pose2d(-11, 13, startingHeading), mirror);
        final Pose2d park1Position = cMirrorY(new Pose2d(-35, 13, startingHeading), mirror);
        final Pose2d park2Position = cMirrorY(new Pose2d(-57, 13, startingHeading), mirror);

        TrajectorySequence park0 = drive.trajectorySequenceBuilder(dropPosition)
                .strafeTo(cMirrorY(new Vector2d(-22,13), mirror))
                .lineToLinearHeading(park0Position)
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(dropPosition)
                .strafeTo(cMirrorY(new Vector2d(-22,13), mirror))
                .lineToLinearHeading(park1Position)
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(dropPosition)
                .strafeTo(cMirrorY(new Vector2d(-22,13), mirror))
                .lineToLinearHeading(park2Position)
                .build();

        AutoTurret turret = new AutoTurret(hardwareMap, 0);
        AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        Thread tool = new Thread(() -> {
            while (opModeIsActive()) {
                tools.update();
            }
        });
        ConeDetector detector = new ConeDetector(hardwareMap, "webcam", true, false);
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
        autoTimer.reset();
        int color = detector.run();
        thread.interrupt();
        if (isStopRequested()) return;
        tool.start();

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        turret.setPos(-34, AutoTurret.Units.DEGREES);
        tools.setIntake(1);
        
        followTrajectorySequence(drive, start);

        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);
        wait(drive, 1000, middlePosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        followTrajectorySequence(drive, middleToHigh);
        // drop 1
        tools.setIntake(-1);
        wait(drive, 500, dropPosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        turret.setPos(54, AutoTurret.Units.DEGREES);
        wait(drive, 200, dropPosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        tools.setPosition(AutoTools.Position.HOVER_5);
        wait(drive, 100, dropPosition);

        followTrajectorySequence(drive, intake);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        tools.setIntake(1);
        wait(drive, 100);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }


        tools.setPosition(AutoTools.Position.INTAKE_5);
        wait(drive, 1000);

        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        tools.setPosition(AutoTools.Position.EXIT_5);
        wait(drive, 1000);

        followTrajectorySequence(drive, intakeToMiddle);

        wait(drive, 500, middlePosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        turret.setPos(-34, AutoTurret.Units.DEGREES);
        wait(drive, 200, middlePosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);

        followTrajectorySequence(drive, middleToHigh);

        wait(drive, 500, dropPosition);
        
        if(isStopRequested() && autoTimer.time(TimeUnit.SECONDS) >= 23) {
            tools.setIntake(0);
            turret.setPos(-100, AutoTurret.Units.DEGREES);
            tools.setPosition(AutoTools.Position.NEUTRAL);
            PoseStorage.currentPose = drive.getPoseEstimate();
            wait(drive, 1000);
            return;
        }

        tools.setIntake(-1);
        wait(drive, 500, dropPosition);

        tools.setIntake(0);

        Pose2d finalPosition;
        switch (color){
            case 0:
                turret.setPos(-100, AutoTurret.Units.DEGREES);
                wait(drive, 1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(drive, 1000);
                turret.setPos(-200, AutoTurret.Units.DEGREES);
                followTrajectorySequence(drive, park0);
                finalPosition = park0Position;
                break;
            case 1:
                turret.setPos(60, AutoTurret.Units.DEGREES);
                wait(drive, 1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(drive, 1000);
                turret.setPos(150, AutoTurret.Units.DEGREES);
                followTrajectorySequence(drive, park1);
                finalPosition = park1Position;
                break;
            case 2:
                turret.setPos(60, AutoTurret.Units.DEGREES);
                wait(drive, 1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(drive, 1000);
                turret.setPos(150, AutoTurret.Units.DEGREES);
                followTrajectorySequence(drive, park2);
                finalPosition = park2Position;
                break;
            default:
                throw new RuntimeException("shut java its initialized");
        }


        

        // for fun :)
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    
    private void followTrajectorySequence(SampleMecanumDrive drive, TrajectorySequence sequence) {
        drive.followTrajectorySequenceAsync(sequence);
        while(!isStopRequested() && autoTimer.time(TimeUnit.SECONDS) < 23 && drive.isBusy()) {
            drive.update();
        }
    }

    private void wait(SampleMecanumDrive drive, int milliseconds, Pose2d pose) {
        timer.reset();
        while (!isStopRequested() && autoTimer.time(TimeUnit.SECONDS) < 23 && timer.time(TimeUnit.MILLISECONDS) < milliseconds) {
            StayInPosition.stayInPose(drive, pose);
        }
    }



    private void wait(SampleMecanumDrive drive, int milliseconds) {
        wait(drive, milliseconds, drive.getPoseEstimate());
    }
}
