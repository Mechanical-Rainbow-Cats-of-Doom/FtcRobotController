package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Timer;

@Autonomous
public class BlueAutoClose extends LinearOpMode {
    public TrajectorySequence sequence;
    public boolean isRed = false;

    public void WaitForDrive(SampleMecanumDrive drive) {
        while(!isStopRequested() && drive.isBusy()) {
            // update the position
            drive.update();
            // keep the current position updated in
            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.addData("pose estimate", drive.getPoseEstimate());
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        final AutoTurret turret = new AutoTurret(hardwareMap);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret);
        Thread thread = new Thread(() -> {
           while (opModeIsActive()) {
               tools.update();
           }
        });
        // start pose
        final Pose2d startPose = cMirrorY(new Pose2d(-35, 63, Math.toRadians(90)), isRed);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);


        // build trajectories
        Pose2d placeEndPose = new Pose2d(-22, 13, Math.toRadians(90));
        Pose2d refillEndPose = new Pose2d(-57, 13, Math.toRadians(90));

        /** Trajectory: Start
         * __________________
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|——|——|—∆|  |  |  |
         *|  |  |  |  |  |  |
         *|—————————————————|
        */
        TrajectorySequence trajectoryStart = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 13, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-22, 13, Math.toRadians(90)))
                .build();

        /** Trajectory: Refill
         * __________________
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  | ⏐|  |  |  |
         *|  |  | ∆|  |  |  |
         *|—————————————————|
         */
        TrajectorySequence trajectoryRefill = drive.trajectorySequenceBuilder(placeEndPose)
                .lineToLinearHeading(new Pose2d(-57, 13, Math.toRadians(90)))
                .build();

        /** Trajectory: Place
         * __________________
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  |  |  |  |  |
         *|  |  | ∆|  |  |  |
         *|  |  | ⏐|  |  |  |
         *|—————————————————|
         */
        TrajectorySequence trajectoryPlace = drive.trajectorySequenceBuilder(refillEndPose)
                .lineToLinearHeading(new Pose2d(-22, 13, Math.toRadians(90)))
                .build();


//        builder.strafeTo(cMirrorY(new Vector2d(-60, 58), isRed));
//
//        builder.strafeTo(cMirrorY(new Vector2d(-57, 13), isRed));
//        builder.strafeTo(cMirrorY(new Vector2d(-22, 13), isRed));
//        builder.waitSeconds(0.5);
//
//        for (int i = 0; i < 2; i++) {
//            builder.strafeTo(cMirrorY(new Vector2d(-57, 13), isRed));
//            builder.waitSeconds(0.5);
//            builder.strafeTo(cMirrorY(new Vector2d(-22, 13), isRed));
//            builder.waitSeconds(0.5);
//        }



        drive.setPoseEstimate(startPose);
        TrajectorySequence wait = drive.trajectorySequenceBuilder(startPose).waitSeconds(10).build();
        waitForStart();
        thread.start();
        // Run delay
        timer.reset();
        DelayStorage.waitForDelay(timer);

        tools.setPosition(AutoTools.Position.NEUTRAL);


        //set to follow the sequence
        drive.followTrajectorySequence(trajectoryStart);
//        WaitForDrive(drive);
        while(!isStopRequested() && drive.isBusy()) {
            // update the position
            drive.update();
            // keep the current position updated in
            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.addData("pose estimate", drive.getPoseEstimate());
            telemetry.update();
        }
        drive.followTrajectorySequence(trajectoryPlace);
        while(!isStopRequested() && drive.isBusy()) {
            // update the position
            drive.update();
            // keep the current position updated in
            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.addData("pose estimate", drive.getPoseEstimate());
            telemetry.update();
        }
//        WaitForDrive(drive);






//        for (int i = 1; i <= 4; i++) {
//            builder.lineToLinearHeading(new Pose2d(-57, 13, Math.toRadians(270)));
//            //end
//            ///tools.setIntake(true);
//            tools.setPosition(AutoTools.Position.INTAKE_NO_INTAKE);
//            //tools.wait();
//            //turret.setPos(50, true);
//            builder.waitSeconds(0.5);
//            builder.lineToLinearHeading(new Pose2d(-22, 13, Math.toRadians(270)));
//            builder.waitSeconds(0.5);
//        }





        // set starting position
        // wait for the start button to be pressed

        // runs until the trajectory sequence is complete

        if (isStopRequested()) return;

        // after the trajectory sequence is complete
        Pose2d finalPose = drive.getPoseEstimate();
        while(!isStopRequested()) {
            StayInPosition.stayInPose(drive, finalPose);
        }
    }
}
